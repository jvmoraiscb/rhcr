/*
© CentraleSupelec, 2017
Author: Dr. Jeremy Fix (jeremy.fix@centralesupelec.fr)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

// Adjustments to new Publication Timing and Execution Framework
// © Siemens AG, 2018, Dr. Martin Bischoff (martin.bischoff@siemens.com)

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class RosPositionVectorSubscriber : UnitySubscriber<MessageTypes.Geometry.Vector3>
    {
        [SerializeField]
        private CarController carController;
        
        private float previousRealTime;
        private Vector3 vector3;
        private bool isMessageReceived;

        protected override void Start()
        {
            base.Start();
        }

        protected override void ReceiveMessage(MessageTypes.Geometry.Vector3 message)
        {
            vector3 = ToVector3(message).Ros2Unity();
            isMessageReceived = true;
        }

        private static Vector3 ToVector3(MessageTypes.Geometry.Vector3 geometryVector3)
        {
            return new Vector3((float)geometryVector3.x, (float)geometryVector3.y, (float)geometryVector3.z);
        }

        private void Update()
        {
            carController.throttle = vector3.y;
            carController.steer = vector3.z;
            if (isMessageReceived)
                ProcessMessage();
            previousRealTime = Time.realtimeSinceStartup;
        }
        private void ProcessMessage()
        {
            float deltaTime = Time.realtimeSinceStartup - previousRealTime;
            isMessageReceived = false;
        }
    }
}