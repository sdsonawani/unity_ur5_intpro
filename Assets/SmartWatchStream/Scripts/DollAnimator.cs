using System;
using System.Collections.Generic;
using System.Linq;
using System.Xml.Linq;
using UnityEngine;

namespace SmartWatchStream.Scripts
{
    public class DollAnimator : MonoBehaviour
    {
        // the scene model to be moved according to motion capture data
        public Avatar destAvatar;
        public GameObject destGameObject;
        public TextAsset motiveSkeletonXML;

        // these are gizmos that are also rendered in-game
        public Mesh gizmoMesh;
        public Material gizmoLineMaterial;
        public Color gizmoColor;
        public bool renderGizmos;
        public bool renderDoll;

        // Lookup map for Motive skeleton bone IDs (keys) and their corresponding GameObjects in Unity (values).
        private Dictionary<string, GameObject> _boneMap = new();
        private GameObject _rootObj;
        private string _skeletonName;

        /// Used in LateUpdate when retrieving and retargeting source pose. Cached and reused for efficiency
        private HumanPose _humanPose;

        // Applies a human pose to an avatar created from Motive XML and CSV GameObject transform hierarchy
        private Avatar _generatedAvatar; // Generated from XML file structure
        private HumanPoseHandler _sourcePoseHandler;

        private HumanPoseHandler
            _destPoseHandler; // The destination of mapping the motive avatar to another Unity model

        // Start is called before the first frame update
        private void Start()
        {
            // First, parse the XML file to get the Motive Skeleton XML structure into Unity ...
            ParseXMLToUnityHierarchy();

            // if the flag was set in the unity editor. Rendering bones will prevent the avatar to be drawn
            if (renderGizmos)
            {
                var root = GetTransformOfMecanimName("Hips");
                RenderSkeletonBone(root);
            }

            if (renderDoll)
                // Now create human pose handlers to map from the generated skeleton to a destination avatar
                CreatePoseHandlers();
            else
                destGameObject.SetActive(false);
        }

        /**
     * All bones back to default position parsed from XML
     */
        public void Reset()
        {
            ParseXMLToUnityHierarchy();
        }

        private void LateUpdate()
        {
            // other managers move the generated skeleton. This late update method maps the positions and rotations
            // of the generated skeleton to the destination avatar.
            if (renderDoll)
                if (_sourcePoseHandler != null && _destPoseHandler != null)
                {
                    // Interpret the streamed pose into Mecanim muscle space representation.
                    _sourcePoseHandler.GetHumanPose(ref _humanPose);

                    // Re-target that muscle space pose to the destination avatar.
                    _destPoseHandler.SetHumanPose(ref _humanPose);
                }

            if (renderGizmos)
            {
                var root = GetTransformOfMecanimName("Hips").gameObject;
                if (root) DrawLines(root);
            }
        }

        private void DrawLines(GameObject root)
        {
            for (var i = 0; i < root.transform.childCount; i++)
            {
                var child = root.transform.GetChild(i).gameObject;
                var lineRenderer = child.GetComponent<LineRenderer>();
                lineRenderer.SetPositions(new[] { child.transform.position, root.transform.position });
                DrawLines(child);
            }
        }

        /**
    * This function parses the input moCapSkeletonXML file and creates a Unity skeleton accordingly.
     * It parses the skeleton name, stored bone IDs, their hierarchy and offsets. The skeleton uses the public
     * _skeletonRootObj as its root object. All bones are also stored in the _boneObjectMap with their ID and GameObject
     * for later lookup.
    */
        private void ParseXMLToUnityHierarchy()
        {
            var moCapSkeleton = XDocument.Parse(motiveSkeletonXML.text);

            // Parse skeleton name from XML
            var nameQuery = from c in moCapSkeleton.Root.Descendants("property")
                where c.Element("name").Value == "NodeName"
                select c.Element("value").Value;
            var skeletonName = nameQuery.First();

            // Parse all bones with parents and offsets
            var bonesQuery = from c in moCapSkeleton.Root.Descendants("bone")
                select (c.Attribute("id").Value,
                    c.Element("offset").Value.Split(","),
                    c.Element("parent_id").Value);

            // create a new object if it doesn't exist yet 
            if (!_rootObj)
            {
                _rootObj = new GameObject("Generated_" + skeletonName);
                // attach it to the dest game object
                var parentTransform = destGameObject.transform;
                _rootObj.transform.SetPositionAndRotation(
                    parentTransform.position,
                    parentTransform.rotation
                );
                _rootObj.transform.parent = parentTransform;
            }

            // recreate Motive skeleton structure in Unity
            foreach (var bone in bonesQuery)
            {
                // transform the XML ID to a Mechanim ID for the lookup map
                var mKey = XmlIDtoMecanimID(bone.Item1);

                // create a new object if it doesn't exist yet (might already exist in case we re-parse the XML)
                if (!_boneMap.ContainsKey(mKey))
                    _boneMap[mKey] = new GameObject(skeletonName + "_" + mKey);
                //the bone with parent 0 is the root object
                _boneMap[mKey].transform.parent = bone.Item3 == "0"
                    ? _rootObj.transform
                    : _boneMap[XmlIDtoMecanimID(bone.Item3)].transform;
                // apply the parsed offsets
                _boneMap[mKey].transform.localPosition = new Vector3(
                    float.Parse(bone.Item2[0]),
                    float.Parse(bone.Item2[1]),
                    float.Parse(bone.Item2[2]));
            }
        }

        private void RenderSkeletonBone(Transform root)
        {
            for (var i = 0; i < root.transform.childCount; i++)
            {
                var child = root.GetChild(i).gameObject;
                child.AddComponent<MeshFilter>().mesh = gizmoMesh;
                var meshRenderer = child.AddComponent<MeshRenderer>();
                meshRenderer.material.color = gizmoColor;

                var lineRenderer = child.AddComponent<LineRenderer>();
                lineRenderer.SetPositions(new[] { child.transform.position, root.transform.position });
                lineRenderer.material = gizmoLineMaterial;
                lineRenderer.startWidth = 0.005f;
                lineRenderer.endWidth = 0.005f;
                lineRenderer.startColor = Color.black;
                lineRenderer.endColor = Color.white;

                RenderSkeletonBone(child.transform);
            }
        }


        public void CreatePoseHandlers()
        {
            // We begin setting up the mapping between Mecanim human anatomy in Unity and the
            // Motive skeleton representation. HumanTrait, SkeletonBone and HumanBone are Unity classes.
            var humanBones = new List<HumanBone>(_boneMap.Count);
            // Set up the T-pose and game object name mappings.
            var skeletonBones = new List<SkeletonBone>(_boneMap.Count + 1);

            // Special case: Create the skeleton root bone
            var rootBone = new SkeletonBone();
            rootBone.name = _rootObj.name;
            rootBone.position = Vector3.zero;
            rootBone.rotation = Quaternion.identity;
            rootBone.scale = Vector3.one;
            skeletonBones.Add(rootBone);

            // now add human bones (Mecanim) and skeleton bones (GameObjects) to their lists
            foreach (var (k, v) in _boneMap)
            {
                // Double-check if mecanim bone name is legit
                if (Array.Find(HumanTrait.BoneName, s => s == k) == "")
                    Debug.Log(k + " is an invalid Mecanim name!");

                // create a Unity HumanBone object
                var humanBone = new HumanBone();
                humanBone.humanName = k;
                humanBone.boneName = v.name;
                humanBone.limit.useDefaultValues = true;
                humanBones.Add(humanBone);

                // create a Unity SkeletonBone object
                var skelBone = new SkeletonBone();
                skelBone.name = v.name;
                skelBone.position = v.transform.localPosition;
                skelBone.rotation = Quaternion.identity;
                skelBone.scale = Vector3.one;
                skeletonBones.Add(skelBone);
            }

            // Now, set up the HumanDescription for the retargeting source Avatar.
            var humanDesc = new HumanDescription();
            // human readable name (humanName) to a skeleton bone name (boneName)
            humanDesc.human = humanBones.ToArray();
            // the skeleton stores the transforms assigned to each bone in T-Pose
            humanDesc.skeleton = skeletonBones.ToArray();

            // These all correspond to default values.
            humanDesc.upperArmTwist = 0.5f;
            humanDesc.lowerArmTwist = 0.5f;
            humanDesc.upperLegTwist = 0.5f;
            humanDesc.lowerLegTwist = 0.5f;
            humanDesc.armStretch = 0.05f;
            humanDesc.legStretch = 0.05f;
            humanDesc.feetSpacing = 0.0f;
            humanDesc.hasTranslationDoF = false;

            // Finally, take the description and build the Avatar for the Motive PoseHandler
            var generatedAvatar = AvatarBuilder.BuildHumanAvatar(_rootObj, humanDesc);

            // confirm everything is correct
            if (generatedAvatar.isValid == false || generatedAvatar.isHuman == false)
            {
                Debug.LogError("Unable to create source Avatar for retargeting. Check that your Skeleton Asset Name " +
                               "and Bone Naming Convention are configured correctly.");
                return;
            }

            // create the HumanPose handlers to map from the Motive Avatar to the Destination Avatar.
            _sourcePoseHandler = new HumanPoseHandler(generatedAvatar, _rootObj.transform);
            _destPoseHandler = new HumanPoseHandler(destAvatar, destGameObject.transform);
        }

        /**
     * IDs in the XML of Motive are distinct from the XML or Mechanim IDs.
     * Use this to map from CSV ID to Unity Mechanim ID.
     */
        public static string XmlIDtoMecanimID(string xmlId)
        {
            var dict = new Dictionary<string, string>()
            {
                { "1", "Hips" },
                { "2", "Spine" },
                { "3", "Chest" },
                { "4", "Neck" },
                { "5", "Head" },

                { "6", "LeftShoulder" },
                { "7", "LeftUpperArm" },
                { "8", "LeftLowerArm" },
                { "9", "LeftHand" },

                { "10", "RightShoulder" },
                { "11", "RightUpperArm" },
                { "12", "RightLowerArm" },
                { "13", "RightHand" },

                { "14", "LeftUpperLeg" },
                { "15", "LeftLowerLeg" },
                { "16", "LeftFoot" },
                { "17", "LeftToes" },

                { "18", "RightUpperLeg" },
                { "19", "RightLowerLeg" },
                { "20", "RightFoot" },
                { "21", "RightToes" }
            };
            return dict[xmlId];
        }

        public Dictionary<string, GameObject> GetBoneMap()
        {
            return _boneMap;
        }

        public Transform GetTransformOfMecanimName(string mecanimName)
        {
            if (_boneMap.ContainsKey(mecanimName))
                return _boneMap[mecanimName].transform;
            return null;
        }

        public Vector3 GetLeftLowerArmVec()
        {
            var diff = GetTransformOfMecanimName("LeftHand").position
                       - GetTransformOfMecanimName("LeftLowerArm").position;
            return diff;
        }


        public Vector3 GetLeftUpperArmVec()
        {
            var diff = GetTransformOfMecanimName("LeftLowerArm").position
                       - GetTransformOfMecanimName("LeftUpperArm").position;
            return diff;
        }

        public float GetLowerArmLength()
        {
            var diff = GetTransformOfMecanimName("LeftLowerArm").position
                       - GetTransformOfMecanimName("LeftHand").position;
            return diff.magnitude;
        }
    }
}