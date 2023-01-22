using UnityEngine;

namespace SmartWatchStream.Scripts
{
    public class RealTimeController : MonoBehaviour
    {
        public DollAnimator swDollAnimator; // the handler to animate the robot
        private SmartWatchSubscriber _sws;
        private int _pastGripperState;

        // Start is called before the first frame update
        private void Start()
        {
            // smartwatch subscriber
            _sws = gameObject.AddComponent<SmartWatchSubscriber>();
        }

        // Update is called once per frame
        private void Update()
        {
            _sws.MoveBoneMap(swDollAnimator.GetBoneMap());

            var gripperState = _sws.GetGripperState();
            if (gripperState != _pastGripperState)
            {
                _pastGripperState = gripperState;
                if (gripperState == 1)
                    Debug.Log("open");
                else
                    Debug.Log("close");
            }
        }
    }
}