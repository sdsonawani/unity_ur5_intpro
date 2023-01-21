/* 
Copyright © 2016 NaturalPoint Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. 
*/

using System;
using UnityEngine;


/// <summary>
/// Implements live tracking of streamed OptiTrack rigid body data onto an object.
/// </summary>
public class OptitrackRigidBody : MonoBehaviour
{
    [Tooltip("The object containing the OptiTrackStreamingClient script.")]
    public OptitrackStreamingClient StreamingClient;

    [Tooltip("The Streaming ID of the rigid body in Motive")]
    public int RigidBodyId;

    [Tooltip("Subscribes to this asset when using Unicast streaming.")]
    public bool NetworkCompensation = true;

    private void Start()
    {
        // If the user didn't explicitly associate a client, find a suitable default.
        if (StreamingClient == null)
        {
            StreamingClient = OptitrackStreamingClient.FindDefaultClient();

            // If we still couldn't find one, disable this component.
            if (StreamingClient == null)
            {
                Debug.LogError(
                    GetType().FullName + ": Streaming client not set, and no " +
                    typeof(OptitrackStreamingClient).FullName + " components found in scene; disabling this component.",
                    this);
                enabled = false;
                return;
            }
        }

        StreamingClient.RegisterRigidBody(this, RigidBodyId);
    }


#if UNITY_2017_1_OR_NEWER
    private void OnEnable()
    {
        Application.onBeforeRender += OnBeforeRender;
    }


    private void OnDisable()
    {
        Application.onBeforeRender -= OnBeforeRender;
    }


    private void OnBeforeRender()
    {
        UpdatePose();
    }
#endif


    private void Update()
    {
        UpdatePose();
    }


    private void UpdatePose()
    {
        var rbState = StreamingClient.GetLatestRigidBodyState(RigidBodyId, NetworkCompensation);
        if (rbState != null)
        {
            transform.localPosition = rbState.Pose.Position;
            transform.localRotation = rbState.Pose.Orientation;
        }
    }
}