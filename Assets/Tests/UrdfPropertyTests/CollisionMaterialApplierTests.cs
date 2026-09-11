using System.Xml;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using UrdfProperties;

/// <summary>
/// URDF に書いた摩擦係数が、実際にコライダへ届いているかを見る。
/// </summary>
/// <remarks>
/// ここが壊れても見た目は何も変わらず、ロボットが滑るという形でしか現れない。
/// 「適用できた数」ではなく個々のコライダの値を読んで確かめる。
///
/// URDF Importer が作る階層 (link/Collisions/&lt;collision ごとの子&gt;/&lt;形状&gt;) を
/// テスト側で組み立てている。Importer 側の構造が変わればここが落ちるので、
/// 黙って摩擦が当たらなくなる事態は防げる。
/// </remarks>
public class CollisionMaterialApplierTests
{
    GameObject robot;

    [TearDown]
    public void TearDown()
    {
        if (robot != null)
        {
            Object.DestroyImmediate(robot);
        }
    }

    static XmlNode RobotNode(string xml)
    {
        var document = new XmlDocument();
        document.LoadXml(xml);
        return document.SelectSingleNode("/robot");
    }

    /// <summary>Importer が作るのと同じ形の link を組み立てる。</summary>
    GameObject BuildLink(string linkName, int collisionCount, int collidersPerCollision = 1)
    {
        var link = new GameObject(linkName);
        var collisions = new GameObject("Collisions");
        collisions.transform.SetParent(link.transform);
        for (int i = 0; i < collisionCount; i++)
        {
            var unnamed = new GameObject($"unnamed_{i}");
            unnamed.transform.SetParent(collisions.transform);
            for (int j = 0; j < collidersPerCollision; j++)
            {
                var shape = new GameObject($"Cylinder_{j}");
                shape.transform.SetParent(unnamed.transform);
                shape.AddComponent<BoxCollider>();
            }
        }
        return link;
    }

    static Collider[] CollidersOf(GameObject link)
    {
        return link.GetComponentsInChildren<Collider>();
    }

    [Test]
    public void Apply_SetsFrictionOnTheCollider()
    {
        robot = BuildLink("wheel_link", 1);
        XmlNode node = RobotNode(@"
            <robot name='probe'>
              <collision_material name='wheel'>
                <friction static='1.0' dynamic='0.8'/>
                <contact_offset value='0.02'/>
              </collision_material>
              <link name='wheel_link'>
                <collision><collision_material name='wheel'/></collision>
              </link>
            </robot>");

        Assert.AreEqual(1, CollisionMaterialApplier.Apply(robot, node));

        Collider collider = CollidersOf(robot)[0];
        Assert.AreEqual(1.0f, collider.material.staticFriction, 1e-4f, "静止摩擦");
        Assert.AreEqual(0.8f, collider.material.dynamicFriction, 1e-4f, "動摩擦");
        Assert.AreEqual(0.02f, collider.contactOffset, 1e-4f, "contact offset");
    }

    [Test]
    public void Apply_DefaultsCombineToAverage()
    {
        robot = BuildLink("wheel_link", 1);
        XmlNode node = RobotNode(@"
            <robot name='probe'>
              <collision_material name='wheel'><friction static='1.0' dynamic='1.0'/></collision_material>
              <link name='wheel_link'><collision><collision_material name='wheel'/></collision></link>
            </robot>");

        CollisionMaterialApplier.Apply(robot, node);

        // 既定は Average。相手のマテリアルとの平均が実効値になるため、
        // static='1.0' と書いても接触相手が 0.6 なら 0.8 で効く。
        Assert.AreEqual(PhysicsMaterialCombine.Average, CollidersOf(robot)[0].material.frictionCombine);
    }

    [Test]
    public void Apply_HonoursCombineAttribute()
    {
        robot = BuildLink("wheel_link", 1);
        XmlNode node = RobotNode(@"
            <robot name='probe'>
              <collision_material name='wheel'>
                <friction static='1.0' dynamic='1.0' combine='maximum'/>
              </collision_material>
              <link name='wheel_link'><collision><collision_material name='wheel'/></collision></link>
            </robot>");

        CollisionMaterialApplier.Apply(robot, node);

        // maximum なら相手によらず指定値が下限になる。
        Assert.AreEqual(PhysicsMaterialCombine.Maximum, CollidersOf(robot)[0].material.frictionCombine);
    }

    [Test]
    public void Apply_CoversEveryColliderOfACollision()
    {
        // 1 つの <collision> が複数コライダに展開されることがある (サブメッシュなど)。
        robot = BuildLink("wheel_link", 1, collidersPerCollision: 3);
        XmlNode node = RobotNode(@"
            <robot name='probe'>
              <collision_material name='wheel'><friction static='0.9' dynamic='0.9'/></collision_material>
              <link name='wheel_link'><collision><collision_material name='wheel'/></collision></link>
            </robot>");

        Assert.AreEqual(3, CollisionMaterialApplier.Apply(robot, node));
        foreach (Collider collider in CollidersOf(robot))
        {
            Assert.AreEqual(0.9f, collider.material.staticFriction, 1e-4f, collider.name);
        }
    }

    [Test]
    public void Apply_MapsEachCollisionToItsOwnObject()
    {
        // 1 リンクに <collision> が 2 つ。順番どおりに別々のマテリアルが当たること。
        robot = BuildLink("foot_link", 2);
        XmlNode node = RobotNode(@"
            <robot name='probe'>
              <collision_material name='grippy'><friction static='1.2' dynamic='1.1'/></collision_material>
              <collision_material name='slippy'><friction static='0.05' dynamic='0.05'/></collision_material>
              <link name='foot_link'>
                <collision><collision_material name='grippy'/></collision>
                <collision><collision_material name='slippy'/></collision>
              </link>
            </robot>");

        Assert.AreEqual(2, CollisionMaterialApplier.Apply(robot, node));

        Transform collisions = robot.transform.Find("Collisions");
        Collider first = collisions.GetChild(0).GetComponentInChildren<Collider>();
        Collider second = collisions.GetChild(1).GetComponentInChildren<Collider>();
        Assert.AreEqual(1.2f, first.material.staticFriction, 1e-4f, "1 つ目の collision");
        Assert.AreEqual(0.05f, second.material.staticFriction, 1e-4f, "2 つ目の collision");
    }

    [Test]
    public void Apply_LeavesContactOffsetAloneWhenNotSpecified()
    {
        robot = BuildLink("wheel_link", 1);
        float before = CollidersOf(robot)[0].contactOffset;
        XmlNode node = RobotNode(@"
            <robot name='probe'>
              <collision_material name='wheel'><friction static='1.0' dynamic='1.0'/></collision_material>
              <link name='wheel_link'><collision><collision_material name='wheel'/></collision></link>
            </robot>");

        CollisionMaterialApplier.Apply(robot, node);

        Assert.AreEqual(before, CollidersOf(robot)[0].contactOffset, 1e-6f,
            "contact_offset を書いていないのに触ってはいけない");
    }

    [Test]
    public void Apply_IgnoresLinksWithoutAReference()
    {
        robot = BuildLink("plain_link", 1);
        XmlNode node = RobotNode(@"
            <robot name='probe'>
              <collision_material name='wheel'><friction static='1.0' dynamic='1.0'/></collision_material>
              <link name='plain_link'><collision/></link>
            </robot>");

        Assert.AreEqual(0, CollisionMaterialApplier.Apply(robot, node));
    }

    [Test]
    public void Apply_UnknownMaterialNameIsNotApplied()
    {
        robot = BuildLink("wheel_link", 1);
        XmlNode node = RobotNode(@"
            <robot name='probe'>
              <collision_material name='wheel'><friction static='1.0' dynamic='1.0'/></collision_material>
              <link name='wheel_link'><collision><collision_material name='typo'/></collision></link>
            </robot>");

        LogAssert.Expect(LogType.Warning, new System.Text.RegularExpressions.Regex("typo"));
        Assert.AreEqual(0, CollisionMaterialApplier.Apply(robot, node));
    }

    [Test]
    public void ParseDefinitions_ReadsFrictionAndOffset()
    {
        XmlNode node = RobotNode(@"
            <robot name='probe'>
              <collision_material name='a'>
                <friction static='0.3' dynamic='0.2' combine='minimum'/>
                <contact_offset value='0.005'/>
              </collision_material>
              <collision_material name='b'><friction static='0.1' dynamic='0.1'/></collision_material>
            </robot>");

        var definitions = CollisionMaterialApplier.ParseDefinitions(node);

        Assert.AreEqual(2, definitions.Count);
        Assert.AreEqual("a", definitions[0].Name);
        Assert.AreEqual(0.3f, definitions[0].StaticFriction, 1e-4f);
        Assert.AreEqual(PhysicsMaterialCombine.Minimum, definitions[0].FrictionCombine);
        Assert.IsTrue(definitions[0].HasContactOffset);
        Assert.AreEqual(0.005f, definitions[0].ContactOffset, 1e-4f);
        Assert.IsFalse(definitions[1].HasContactOffset, "contact_offset の無い定義");
    }

    [Test]
    public void Apply_SensorOnlyMakesEveryColliderATrigger()
    {
        // 雑草: LiDAR には映るが、触れても押し返さない。トリガなら物理接触は起きず
        // レイキャストには当たる (Queries Hit Triggers は有効)。
        robot = BuildLink("weeds_link", 1, collidersPerCollision: 2);
        XmlNode node = RobotNode(@"
            <robot name='weeds'>
              <collision_material name='weed'><sensor_only value='true'/></collision_material>
              <link name='weeds_link'><collision><collision_material name='weed'/></collision></link>
            </robot>");

        Assert.AreEqual(2, CollisionMaterialApplier.Apply(robot, node));
        foreach (Collider collider in CollidersOf(robot))
        {
            Assert.IsTrue(collider.isTrigger, collider.name);
        }
        Assert.IsTrue(CollisionMaterialApplier.AllCollidersAreSensorOnly(robot));
    }

    [Test]
    public void Apply_LeavesIsTriggerAloneWhenNotSpecified()
    {
        // 摩擦だけの定義で、うっかりトリガにしてはいけない (ロボットが床を抜ける)。
        robot = BuildLink("wheel_link", 1);
        XmlNode node = RobotNode(@"
            <robot name='probe'>
              <collision_material name='wheel'><friction static='1.0' dynamic='1.0'/></collision_material>
              <link name='wheel_link'><collision><collision_material name='wheel'/></collision></link>
            </robot>");

        CollisionMaterialApplier.Apply(robot, node);

        Assert.IsFalse(CollidersOf(robot)[0].isTrigger);
        Assert.IsFalse(CollisionMaterialApplier.AllCollidersAreSensorOnly(robot));
    }

    [Test]
    public void Apply_SensorOnlyAppliesOnlyToTheReferencingCollision()
    {
        // 同じリンクに「実体」と「雑草」が混在しても、参照した collision だけがトリガになる。
        robot = BuildLink("mixed_link", 2);
        XmlNode node = RobotNode(@"
            <robot name='probe'>
              <collision_material name='solid'><friction static='1.0' dynamic='1.0'/></collision_material>
              <collision_material name='weed'><sensor_only/></collision_material>
              <link name='mixed_link'>
                <collision><collision_material name='solid'/></collision>
                <collision><collision_material name='weed'/></collision>
              </link>
            </robot>");

        Assert.AreEqual(2, CollisionMaterialApplier.Apply(robot, node));

        Transform collisions = robot.transform.Find("Collisions");
        Assert.IsFalse(collisions.GetChild(0).GetComponentInChildren<Collider>().isTrigger, "solid");
        Assert.IsTrue(collisions.GetChild(1).GetComponentInChildren<Collider>().isTrigger, "weed");
        Assert.IsFalse(CollisionMaterialApplier.AllCollidersAreSensorOnly(robot));
    }

    [Test]
    public void Apply_SensorOnlyConvexifiesANonConvexMeshCollider()
    {
        // Unity は非 convex の MeshCollider をトリガにできない。黙って効かないより
        // convex に変えて (警告付きで) トリガにする。
        robot = new GameObject("weeds_link");
        var collisions = new GameObject("Collisions");
        collisions.transform.SetParent(robot.transform);
        var unnamed = new GameObject("unnamed_0");
        unnamed.transform.SetParent(collisions.transform);
        var shape = new GameObject("Mesh_0");
        shape.transform.SetParent(unnamed.transform);
        var meshCollider = shape.AddComponent<MeshCollider>();
        GameObject donor = GameObject.CreatePrimitive(PrimitiveType.Cube);
        meshCollider.sharedMesh = donor.GetComponent<MeshFilter>().sharedMesh;
        Object.DestroyImmediate(donor);
        meshCollider.convex = false;

        XmlNode node = RobotNode(@"
            <robot name='weeds'>
              <collision_material name='weed'><sensor_only/></collision_material>
              <link name='weeds_link'><collision><collision_material name='weed'/></collision></link>
            </robot>");

        LogAssert.Expect(LogType.Warning, new System.Text.RegularExpressions.Regex("convex"));
        Assert.AreEqual(1, CollisionMaterialApplier.Apply(robot, node));
        Assert.IsTrue(meshCollider.convex, "convex にしてから");
        Assert.IsTrue(meshCollider.isTrigger, "トリガにする");
    }

    [Test]
    public void ParseDefinitions_ReadsSensorOnly()
    {
        XmlNode node = RobotNode(@"
            <robot name='probe'>
              <collision_material name='bare'><sensor_only/></collision_material>
              <collision_material name='explicit'><sensor_only value='true'/></collision_material>
              <collision_material name='negated'><sensor_only value='false'/></collision_material>
              <collision_material name='absent'><friction static='0.1' dynamic='0.1'/></collision_material>
            </robot>");

        var definitions = CollisionMaterialApplier.ParseDefinitions(node);

        Assert.AreEqual(4, definitions.Count);
        Assert.IsTrue(definitions[0].SensorOnly, "要素だけで true");
        Assert.IsTrue(definitions[1].SensorOnly, "value='true'");
        Assert.IsFalse(definitions[2].SensorOnly, "value='false' で打ち消し");
        Assert.IsFalse(definitions[3].SensorOnly, "要素が無ければ false");
    }

    [Test]
    public void ParseDefinitions_AcceptsTheDeprecatedElementName()
    {
        XmlNode node = RobotNode(@"
            <robot name='probe'>
              <physics_material name='old'><friction static='0.7' dynamic='0.7'/></physics_material>
            </robot>");

        LogAssert.Expect(LogType.Warning, new System.Text.RegularExpressions.Regex("deprecated"));
        var definitions = CollisionMaterialApplier.ParseDefinitions(node);

        Assert.AreEqual(1, definitions.Count);
        Assert.AreEqual(0.7f, definitions[0].StaticFriction, 1e-4f);
    }
}
