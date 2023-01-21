using UnityEngine;

namespace SetupController
{
    public class RealTimeController : MonoBehaviour
    {
        public SmartWatchStream.DollAnimator swDollAnimator; // the handler to animate the robot
        private SmartWatchSubscriber _sws;
        private int _pastGripperState = 0;
        public GameObject thumb;

        // Start is called before the first frame update
        private void Start()
        {
            // smartwatch subscriber
            _sws = this.AddComponent<SmartWatchSubscriber>();
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
                {
                    Debug.Log("open");
                    thumb.transform.Translate(new Vector3(-0.07f, 0, 0));
                }
                else
                {
                    Debug.Log("close");
                    thumb.transform.Translate(new Vector3(0.07f, 0, 0));
                }
            }
        }
    }
}