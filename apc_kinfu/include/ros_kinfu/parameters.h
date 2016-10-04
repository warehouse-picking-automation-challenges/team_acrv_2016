/*
 * Copyright (c) 2013-2015, Riccardo Monica
 *   RIMLab, Department of Information Engineering, University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other materials provided with
 * the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PARAMETERS_H
#define PARAMETERS_H

// Input ROS topics
#define PARAM_NAME_PREFIX_TOPIC           "prefix_topic"
#define PARAM_DEFAULT_PREFIX_TOPIC        "/camera"

#define PARAM_NAME_DEPTH_IMAGE_TOPIC      "depth_image_topic"
#define PARAM_DEFAULT_DEPTH_IMAGE_TOPIC   "/depth/image_raw"

#define PARAM_NAME_CAMERA_INFO_TOPIC      "camera_info_topic"
#define PARAM_DEFAULT_CAMERA_INFO_TOPIC   "/depth/camera_info"

#define PARAM_NAME_IMAGE_TOPIC            "image_topic"
#define PARAM_DEFAULT_IMAGE_TOPIC         "/rgb/image_color"


// kinfu parameters
#define PARAM_NAME_VOLUME_SIZE            "volume_size"
#define PARAM_SNAME_VOLUME_SIZE           "vs"
#define PARAM_DEFAULT_VOLUME_SIZE         (double(3.0))

#define PARAM_NAME_SHIFT_DISTANCE         "shift_distance"
#define PARAM_SNAME_SHIFT_DISTANCE        "sd"
#define PARAM_DEFAULT_SHIFT_DISTANCE      (double(1.5))

#define PARAM_NAME_SNAPSHOT_RATE          "snapshot_rate"
#define PARAM_SNAME_SNAPSHOT_RATE         "sr"
#define PARAM_DEFAULT_SNAPSHOT_RATE       (int(45))

#define PARAM_NAME_CUDA_DEVICE_ID         "cuda_device_id"
#define PARAM_DEFAULT_CUDA_DEVICE_ID      (int(0))

#define PARAM_NAME_EXTRACT_TEXTURES       "extract_textures"
#define PARAM_SNAME_EXTRACT_TEXTURES      "et"
#define PARAM_DEFAULT_EXTRACT_TEXTURES    (bool(false))

#define PARAM_NAME_DEPTH_WIDTH            "depth_width"
#define PARAM_DEFAULT_DEPTH_WIDTH         640

#define PARAM_NAME_DEPTH_HEIGHT           "depth_height"
#define PARAM_DEFAULT_DEPTH_HEIGHT        480


#endif // PARAMETERS_H
