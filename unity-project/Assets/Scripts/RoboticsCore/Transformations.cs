// Copyright 2019-2021 Robotec.ai.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using UnityEngine;
using RosMessageTypes.Geometry;

namespace Unity.Robotics.Core{
    public static class Transformations{
        public static Vector3 Ros2Unity(this PointMsg msg){
            return new Vector3(-(float)msg.y, (float)msg.z, (float)msg.x);
        }

        public static PointMsg Unity2Ros(this Vector3 vector3){
            return new PointMsg(vector3.z, -vector3.x, vector3.y);
        }

        public static Quaternion Ros2Unity(this QuaternionMsg msg){
            return new Quaternion((float)msg.y, (float)-msg.z, (float)-msg.x, (float)msg.w);
        }

        public static QuaternionMsg Unity2Ros(this Quaternion quaternion){
            return new QuaternionMsg(-quaternion.z, quaternion.x, -quaternion.y, quaternion.w);
        }
    }
}