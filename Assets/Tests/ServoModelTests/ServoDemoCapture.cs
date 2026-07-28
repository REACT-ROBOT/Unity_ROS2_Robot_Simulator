using System.Collections;
using System.IO;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

/// <summary>
/// Renders the servo-model pendulum demo to a PNG frame sequence for video
/// production. Marked [Explicit] so normal test runs skip it; run with
///   Unity -batchmode -runTests -testPlatform playmode -testFilter ServoDemoCapture
/// (WITHOUT -nographics — a graphics device is required).
/// Frames go to <project>/TestOutput/frames_<name>/; encode with ffmpeg.
///
/// Choreography per scenario (26 s, captured at 25 fps):
///   0-2 s   hold at -0.6 rad (settle)
///   2-12 s  constant-velocity sweep -0.6 -> +0.6 rad (gravity crossing)
///   12-24 s triangle wave +-0.5 rad, 2 cycles (command reversals)
///   24-26 s hold (stiction)
/// The translucent orange arm shows the commanded angle; the solid blue arm is
/// the physical joint driven through the servo model.
/// </summary>
public class ServoDemoCapture
{
    const int W = 960, H = 540;

    Material MakeMat(Color c, bool transparent = false)
    {
        var mat = new Material(Shader.Find(transparent
            ? "Legacy Shaders/Transparent/Diffuse" : "Standard"));
        mat.color = c;
        return mat;
    }

    GameObject MakeArmVisual(Transform parent, Material mat, string name)
    {
        var root = new GameObject(name);
        root.transform.SetParent(parent, false);
        var rod = GameObject.CreatePrimitive(PrimitiveType.Cube);
        Object.DestroyImmediate(rod.GetComponent<Collider>());
        rod.transform.SetParent(root.transform, false);
        rod.transform.localPosition = new Vector3(0f, -0.11f, 0f);
        rod.transform.localScale = new Vector3(0.022f, 0.22f, 0.022f);
        rod.GetComponent<Renderer>().material = mat;
        var bob = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        Object.DestroyImmediate(bob.GetComponent<Collider>());
        bob.transform.SetParent(root.transform, false);
        bob.transform.localPosition = new Vector3(0f, -0.2f, 0f);
        bob.transform.localScale = Vector3.one * 0.055f;
        bob.GetComponent<Renderer>().material = mat;
        return root;
    }

    [UnityTest, Explicit, Category("Capture")]
    public IEnumerator CaptureDemoVideos()
    {
        yield return CaptureScenario("real1deg", 0.0175f,
            "backlash 1 deg (real params)");
        yield return CaptureScenario("exag10deg", 0.1745f,
            "backlash 10 deg (exaggerated)");
    }

    IEnumerator CaptureScenario(string name, float gapWidth, string caption)
    {
        Time.fixedDeltaTime = 0.02f;
        Time.timeScale = 1f;
        Time.maximumDeltaTime = 1f;
        Physics.gravity = new Vector3(0f, -9.81f, 0f);

        // --- physics rig (same as ServoPendulumTests) ---
        var root = new GameObject("demo_base");
        var baseBody = root.AddComponent<ArticulationBody>();
        baseBody.immovable = true;
        var arm = new GameObject("demo_arm");
        arm.transform.SetParent(root.transform, false);
        var ab = arm.AddComponent<ArticulationBody>();
        ab.jointType = ArticulationJointType.RevoluteJoint;
        ab.twistLock = ArticulationDofLock.FreeMotion;
        ab.anchorPosition = Vector3.zero;
        ab.anchorRotation = Quaternion.identity;
        ab.mass = 0.2f;
        ab.automaticCenterOfMass = false;
        ab.centerOfMass = new Vector3(0f, -0.2f, 0f);
        ab.automaticInertiaTensor = false;
        ab.inertiaTensor = new Vector3(2e-3f, 2e-3f, 2e-3f);
        ab.inertiaTensorRotation = Quaternion.identity;
        ab.useGravity = true;

        var servo = arm.AddComponent<ServoJointModel>();
        servo.servoStiffness = 20f;
        servo.servoDamping = 0.5f;
        servo.motorTorqueLimit = 2f;
        servo.motorInertia = 2e-3f;
        servo.staticFriction = 0.15f;
        servo.dynamicFriction = 0.08f;
        servo.stribeckVelocity = 0.1f;
        servo.viscousFriction = 0.005f;
        servo.backlashWidth = gapWidth;
        servo.transmissionStiffness = 400f;
        servo.transmissionDamping = 0.5f;

        // --- visuals ---
        var blue = MakeMat(new Color32(0x2a, 0x78, 0xd6, 0xff));
        var orange = MakeMat(new Color32(0xff, 0x86, 0x45, 0xc8), transparent: true);
        var gray = MakeMat(new Color32(0xc3, 0xc2, 0xb7, 0xff));

        MakeArmVisual(arm.transform, blue, "arm_visual");
        var ghost = new GameObject("cmd_ghost");
        MakeArmVisual(ghost.transform, orange, "ghost_visual");

        var stand = GameObject.CreatePrimitive(PrimitiveType.Cube);
        Object.DestroyImmediate(stand.GetComponent<Collider>());
        stand.transform.position = new Vector3(0f, 0.045f, 0f);
        stand.transform.localScale = new Vector3(0.09f, 0.05f, 0.09f);
        stand.GetComponent<Renderer>().material = gray;
        var hub = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        Object.DestroyImmediate(hub.GetComponent<Collider>());
        hub.transform.position = Vector3.zero;
        hub.transform.localScale = Vector3.one * 0.035f;
        hub.GetComponent<Renderer>().material = gray;

        var lightGo = new GameObject("demo_light");
        var light = lightGo.AddComponent<Light>();
        light.type = LightType.Directional;
        light.intensity = 1.3f;
        lightGo.transform.rotation = Quaternion.Euler(35f, 160f, 0f);

        var camGo = new GameObject("demo_cam");
        var cam = camGo.AddComponent<Camera>();
        camGo.transform.position = new Vector3(0.88f, -0.1f, 0f);
        camGo.transform.rotation = Quaternion.Euler(0f, -90f, 0f);
        cam.fieldOfView = 33f;
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.backgroundColor = new Color32(0xf2, 0xf1, 0xec, 0xff);

        var font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        TextMesh MakeText(Vector3 pos, int size, Color col, TextAnchor anchor)
        {
            var go = new GameObject("label");
            go.transform.position = pos;
            go.transform.rotation = Quaternion.Euler(0f, -90f, 0f);
            var tm = go.AddComponent<TextMesh>();
            tm.font = font;
            go.GetComponent<MeshRenderer>().material = font.material;
            go.GetComponent<MeshRenderer>().material.color = col;
            tm.fontSize = size;
            tm.characterSize = 0.01f;
            tm.anchor = anchor;
            return tm;
        }
        var title = MakeText(new Vector3(0f, 0.145f, -0.44f), 27, new Color32(0x3a, 0x3a, 0x35, 0xff), TextAnchor.UpperLeft);
        title.text = "ServoJointModel demo — " + caption;
        var phase = MakeText(new Vector3(0f, 0.11f, -0.44f), 26, new Color32(0x8a, 0x8a, 0x80, 0xff), TextAnchor.UpperLeft);
        var legend = MakeText(new Vector3(0f, -0.325f, -0.44f), 24, new Color32(0x3a, 0x3a, 0x35, 0xff), TextAnchor.UpperLeft);
        legend.text = "orange = command   blue = joint (physics)";

        // --- capture setup ---
        string dir = Path.Combine(Application.dataPath, "..", "TestOutput", "frames_" + name);
        Directory.CreateDirectory(dir);
        var rt = new RenderTexture(W, H, 24);
        var tex = new Texture2D(W, H, TextureFormat.RGB24, false);
        cam.targetTexture = rt;

        int frameNo = 0;
        void Capture()
        {
            cam.Render();
            RenderTexture.active = rt;
            tex.ReadPixels(new Rect(0, 0, W, H), 0, 0);
            tex.Apply();
            File.WriteAllBytes(Path.Combine(dir, $"frame_{frameNo:D4}.png"), tex.EncodeToPNG());
            frameNo++;
        }

        // --- choreography ---
        const float dt = 0.02f;
        int total = Mathf.RoundToInt(26f / dt);
        for (int i = 0; i < total; i++)
        {
            float t = i * dt;
            float cmd, vel;
            if (t < 2f) { cmd = -0.6f; vel = 0f; phase.text = "hold"; }
            else if (t < 12f)
            {
                vel = 0.12f; cmd = -0.6f + vel * (t - 2f);
                phase.text = "constant-velocity sweep through gravity crossing";
            }
            else if (t < 24f)
            {
                float tt = (t - 12f) / 6f - Mathf.Floor((t - 12f) / 6f);
                float a = 0.5f;
                if (tt < 0.25f) { cmd = 0.6f - (0.6f - a) - 4f * a * (0.25f - tt); vel = 4f * a / 6f; }
                else if (tt < 0.75f) { cmd = a - 4f * a * (tt - 0.25f); vel = -4f * a / 6f; }
                else { cmd = -a + 4f * a * (tt - 0.75f); vel = 4f * a / 6f; }
                phase.text = "triangle wave: watch the lag on each reversal";
            }
            else { cmd = 0f; vel = 0f; phase.text = "hold (stiction)"; }

            servo.SetCommand(cmd, vel);
            ghost.transform.rotation = Quaternion.Euler(cmd * Mathf.Rad2Deg, 0f, 0f);
            yield return new WaitForFixedUpdate();
            if (i % 2 == 0)
                Capture();
        }

        Assert.Greater(frameNo, 300, "not enough frames captured");
        cam.targetTexture = null;
        Object.Destroy(root);
        Object.Destroy(ghost);
        Object.Destroy(camGo);
        Object.Destroy(lightGo);
        Object.Destroy(stand);
        Object.Destroy(hub);
        Object.Destroy(title.gameObject);
        Object.Destroy(phase.gameObject);
        Object.Destroy(legend.gameObject);
        Object.Destroy(rt);
    }
}
