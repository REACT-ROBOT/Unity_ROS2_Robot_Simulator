using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

/// <summary>
/// Where Unity delivers collision callbacks when several colliders share one
/// ArticulationBody, and what a ContactPoint says about which of them was touched.
/// </summary>
/// <remarks>
/// This decides how a per-part contact sensor has to be wired. Merging fixed URDF links
/// into their parent body (needed to stay under PhysX's 64-body articulation limit)
/// leaves the parts as collider-only children, so a sensor sitting on the part receives
/// nothing if Unity only talks to the body. Filtering by ContactPoint.thisCollider is
/// only viable if that field actually names the child collider.
/// </remarks>
public class ContactRoutingTests
{
    private class Recorder : MonoBehaviour
    {
        public int enterCalls;
        public readonly List<int> contactCounts = new List<int>();
        public readonly HashSet<Collider> thisColliders = new HashSet<Collider>();
        public readonly HashSet<Collider> otherColliders = new HashSet<Collider>();

        void OnCollisionEnter(Collision c) => Record(c);
        void OnCollisionStay(Collision c) => Record(c);

        private void Record(Collision c)
        {
            enterCalls++;
            contactCounts.Add(c.contactCount);
            otherColliders.Add(c.collider);
            for (int i = 0; i < c.contactCount; i++)
            {
                thisColliders.Add(c.GetContact(i).thisCollider);
            }
        }
    }

    private GameObject m_Root;
    private GameObject m_Ball;

    [UnityTest]
    public IEnumerator CollisionsOnMergedBody_ReportWhichChildColliderWasTouched()
    {
        // A body with two collider-only children, standing in for two armour plates
        // merged onto one link.
        m_Root = new GameObject("body");
        m_Root.transform.position = Vector3.zero;
        var body = m_Root.AddComponent<ArticulationBody>();
        body.immovable = true;

        Collider left = MakePlate("left", new Vector3(-1.0f, 0f, 0f));
        Collider right = MakePlate("right", new Vector3(1.0f, 0f, 0f));

        var rootRecorder = m_Root.AddComponent<Recorder>();
        var leftRecorder = left.gameObject.AddComponent<Recorder>();
        var rightRecorder = right.gameObject.AddComponent<Recorder>();

        // Drop a ball onto the left plate only.
        m_Ball = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        m_Ball.transform.position = new Vector3(-1.0f, 1.2f, 0f);
        m_Ball.transform.localScale = Vector3.one * 0.3f;
        var ballBody = m_Ball.AddComponent<Rigidbody>();
        ballBody.mass = 1f;

        for (int i = 0; i < 400; i++)
        {
            if (rootRecorder.enterCalls > 0 || leftRecorder.enterCalls > 0) break;
            yield return new WaitForFixedUpdate();
        }
        // let a few more steps accumulate contacts
        for (int i = 0; i < 20; i++) yield return new WaitForFixedUpdate();

        Debug.Log($"[ContactRouting] body GameObject: {rootRecorder.enterCalls} calls, " +
                  $"contactCounts=[{string.Join(",", rootRecorder.contactCounts)}], " +
                  $"thisColliders=[{Names(rootRecorder.thisColliders)}], " +
                  $"otherColliders=[{Names(rootRecorder.otherColliders)}]");
        Debug.Log($"[ContactRouting] left plate GameObject:  {leftRecorder.enterCalls} calls");
        Debug.Log($"[ContactRouting] right plate GameObject: {rightRecorder.enterCalls} calls");

        Assert.Greater(rootRecorder.enterCalls, 0,
            "the body's GameObject should hear about collisions of its child colliders");
        Assert.Contains(left, new List<Collider>(rootRecorder.thisColliders),
            "ContactPoint.thisCollider should name the child collider that was touched");
        Assert.IsFalse(rootRecorder.thisColliders.Contains(right),
            "the untouched plate must not appear, otherwise per-part filtering is impossible");
    }

    private static Collider MakePlate(string name, Vector3 localPosition)
    {
        var go = new GameObject(name);
        go.transform.SetParent(GameObject.Find("body").transform, false);
        go.transform.localPosition = localPosition;
        var col = go.AddComponent<BoxCollider>();
        col.size = new Vector3(0.6f, 0.1f, 0.6f);
        return col;
    }

    private static string Names(IEnumerable<Collider> colliders)
    {
        var names = new List<string>();
        foreach (var c in colliders) names.Add(c == null ? "null" : c.name);
        return string.Join(",", names);
    }

    [TearDown]
    public void TearDown()
    {
        if (m_Root != null) Object.DestroyImmediate(m_Root);
        if (m_Ball != null) Object.DestroyImmediate(m_Ball);
    }
}
