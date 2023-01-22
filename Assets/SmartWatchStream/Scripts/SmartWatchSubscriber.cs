using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

namespace SmartWatchStream.Scripts
{
    public class SmartWatchSubscriber : MonoBehaviour
    {
        private ROSConnection _ros;
        private GameObject _leftHandObj;
        private GameObject _leftLowerArmObj;
        private GameObject _leftUpperArmObj;
        private float _gripperState = 0f;

        // Start is called before the first frame update
        private void Start()
        {
            // these are placeholders moved by the SmartWatchCallback
            _leftHandObj = new GameObject("SwLeftHand");
            _leftLowerArmObj = new GameObject("SwLeftLowerArm");
            _leftUpperArmObj = new GameObject("SwLeftUpperArm");

            // Using the Ros plugin to subscribe to the smartwatch channel
            _ros = ROSConnection.GetOrCreateInstance();
            _ros.Subscribe<Float32MultiArrayMsg>("/smartwatch_stream", SmartWatchCallback);
        }

        public void MoveBoneMap(Dictionary<string, GameObject> boneMap)
        {
            var hPos = boneMap["Hips"].transform.position;

            boneMap["LeftHand"].transform.SetPositionAndRotation(
                _leftHandObj.transform.position + hPos,
                _leftHandObj.transform.rotation
            );
            boneMap["LeftLowerArm"].transform.SetPositionAndRotation(
                _leftLowerArmObj.transform.position + hPos,
                _leftLowerArmObj.transform.rotation
            );
            boneMap["LeftUpperArm"].transform.rotation = _leftUpperArmObj.transform.rotation;
        }

        public int GetGripperState()
        {
            return (int)_gripperState;
        }

        private void SmartWatchCallback(Float32MultiArrayMsg msg)
        {
            // read wrist position and rotation from message
            var pos = new Vector3(msg.data[0], msg.data[1], msg.data[2]);
            var rot = new Quaternion(w: msg.data[3], x: msg.data[4], y: msg.data[5], z: msg.data[6]);
            _leftHandObj.transform.SetPositionAndRotation(pos, rot);

            // read elbow position and rotation from message
            var ePos = new Vector3(msg.data[7], msg.data[8], msg.data[9]);
            var eRot = new Quaternion(w: msg.data[10], x: msg.data[11], y: msg.data[12], z: msg.data[13]);
            _leftLowerArmObj.transform.SetPositionAndRotation(ePos, eRot);

            // read shoulder position and rotation from message
            var sPos = new Vector3(msg.data[14], msg.data[15], msg.data[16]);
            var sRot = new Quaternion(w: msg.data[17], x: msg.data[18], y: msg.data[19], z: msg.data[20]);
            _leftUpperArmObj.transform.SetPositionAndRotation(sPos, sRot);


            _gripperState = msg.data[21];
        }
    }
}