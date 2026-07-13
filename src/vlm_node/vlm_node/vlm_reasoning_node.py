#!/usr/bin/env python3
# coding: utf-8
import json
import rclpy
import base64
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image as PILImage
import torch
from torchvision import transforms
import cv2
import numpy as np
from vlm_node.constants import (
    VLM_PROVIDER, VLM_API_KEY, VLM_BASE_URL, MODEL_NAME, MODEL_NAME_LITE,
)
from tare_planner.msg import RoomType, ObjectType
from openai import OpenAI
from pydantic import BaseModel
import os
import json
import time
from collections import deque
import threading
import yaml
from rclpy.time import Time
import yaml


class VLMNode(Node):
    def __init__(self):
        super().__init__('vlm_node')

        # Initialize VLM
        self.declare_parameter('log_dir', 'logs/episode_0')

        self.log_dir = self.get_parameter('log_dir').get_parameter_value().string_value

        self.get_logger().info(f"Log directory: {self.log_dir}")

        try:
            self.vlm_model = OpenAI(
                api_key=VLM_API_KEY,
                base_url=VLM_BASE_URL,
            )
            self.get_logger().info(f"✅ VLM initialized ({VLM_PROVIDER})")
        except Exception as e:
            self.get_logger().error(f"❌ VLM initialization failed: {e}")
            return

        self.room_type_vlm_model = MODEL_NAME
        self.object_type_vlm_model = MODEL_NAME_LITE
        
        # queues
        self.room_type_query_queue = deque()
        self.object_type_query_queue = deque()

        # Simulation room types
        # self.room_types = ["Living Room", "Bedroom", "Kitchen", "Bathroom", "Balcony", "Garden"]
        # Gates 4th floor room_types
        # self.room_types = ["Classroom", "Office Room", "Computer Lab", "Restroom", "Student Lounge", "Reception", "Corridor"]
        # Gates 5th floor room_types
        # self.room_types = ["Classroom", "Office Room", "Meeting Room", "Computer Lab", "Restroom", "Storage Room", "Copy Room", "Student Lounge", "Reception", "Corridor"]
        # self.room_types = ["Classroom", "Computer Lab", "Restroom", "Student Lounge", "Corridor"]
        # NSH room_types
        # self.room_types = ["Classroom", "Laboratory", "Office Room", "Meeting Room", "Computer Lab", "Restroom", "Storage Room", "Copy Room", "Student Lounge", "Reception", "Corridor"]
        # self.room_types = ["Office Room"]
        # CIC room_types
        # self.room_types = ["Office Room", "Meeting Room", "Open Workspace", "Interview Room", "Reception", "Print Room", "Storage Room", "Restroom"]
        # AlphaZ room types
        self.room_types = ["Office Room", "Meeting Room", "Kitchenette", "Lobby", "Tool Room"]
        self.ROOM_TYPE_PROMPT = """
        You are given an rgb image of a room and a top-down room layout mask image.
        Identify the room type and respond strictly in valid JSON format.

        Use the key "room_type" and select one of the options listed below as the value.  
        For example:
        {"room_type": "Living Room"}

        Options:
        """
        self.ROOM_TYPE_PROMPT_FREE = """
        You are given an rgb image of a room and a top-down room layout mask image.
        Identify the room type and respond strictly in valid JSON format.

        Use the key "room_type" and use your own knowledge to determine the room type.
        The room type should be a single word or a short phrase (no more than three words) that accurately describes the room.
        Do not use vague or generic terms like "room" or "area". 
        Do not use words like "undetermined" or "unknown". You can make a reasonable guess based on the visual features of the room.
        For example:
        {"room_type": "Living Room"}
        """

        self.object_type_query_prompt = """
        You are provided with an RGB image containing an object within a bounding box, along with a list of candidate labels. Your task is to determine the correct label for the object based on the image and select the most appropriate option from the list. The response must be strictly in valid JSON format.
        Use the key "label" and select one of the provided labels as the value.  
        For example:
        {"label": "Chair"}
        """

        object_file_path = 'src/semantic_mapping/semantic_mapping/config/objects.yaml'
        with open(object_file_path, "r") as file:
            self.object_config = yaml.safe_load(file)
        self.label_template = self.object_config['prompts']
        self.object_list = []
        for value in self.label_template.values():
            self.object_list += value['prompts']
        self.get_logger().info(f"Object List: {self.object_list}")

        self.bridge = CvBridge()

        # ----------------------- Subscribers -----------------------
        # Subscriber: receive the room type query
        self.room_type_query_subscription = self.create_subscription(
            RoomType,
            '/room_type_query',
            self.room_type_callback,
            10
        )

        self.object_type_query_subscription = self.create_subscription(
            ObjectType,
            '/object_type_query',
            self.object_type_query_callback,
            50
        )
        
        # ----------------------- End of Subscribers ---------------------

        # ----------------------- Publishers -----------------------
        # Publisher: publish room type answer
        self.room_type_publisher = self.create_publisher(
            RoomType,
            '/room_type_answer',
            10
        )

        self.object_type_answer_publisher = self.create_publisher(
            ObjectType,
            '/object_type_answer',
            50
        )
        
        # ----------------------- End of Publishers ---------------------

        self.mapping_timer = self.create_timer(0.1, self.vlm_node_callback)

        # debug
        os.system("rm -rf debug")
        os.makedirs("debug", exist_ok=True)
        os.makedirs("debug/room_type", exist_ok=True)
        os.makedirs("debug/object_type", exist_ok=True)
        os.makedirs("debug/img_lidar", exist_ok=True)

        self.get_logger().info("🚀 VLM Node started")
        
    
    def room_type_callback(self, msg: RoomType):
        self.room_type_query_queue.append(msg)

    def object_type_query_callback(self, msg: ObjectType):
        self.object_type_query_queue.append(msg)
    
    def process_room_type_query(self, msg: RoomType):
        """Handle room type query and publish answer"""
        class Step(BaseModel):
            explanation: str
            output: str

        class Result(BaseModel):
            # steps: list[Step]
            room_type: str
        try:
            # Process the room type query
            # self.get_logger().info(f"Received room type query: {msg}")
            start_time = time.time()
            # Load the best-3 room images from disk (paths set by the planner).
            image_b64_list = []
            for path in msg.image_paths:
                cv_img = cv2.imread(path)
                if cv_img is None:
                    self.get_logger().warn(f"Could not read room image: {path}")
                    continue
                image_b64_list.append(
                    base64.b64encode(cv2.imencode('.jpg', cv_img)[1]).decode('utf-8'))
            cv_room_mask = None
            room_mask_base64 = None
            if msg.room_mask.data:
                cv_room_mask = self.bridge.imgmsg_to_cv2(msg.room_mask, desired_encoding='mono8')
                room_mask_base64 = base64.b64encode(
                    cv2.imencode('.jpg', cv_room_mask)[1]).decode('utf-8')
            if not image_b64_list and not msg.objects:
                raise ValueError("Room type query has neither images nor objects")
            # if msg.in_room:
            #     room_type_prompt = self.ROOM_TYPE_PROMPT
            #     room_types = self.room_types.copy()
            #     for i, room_type in enumerate(room_types):
            #         room_type_prompt += f"{i}. {room_type}\n"
            # else:
            #     room_type_prompt = self.ROOM_TYPE_PROMPT
            #     room_types = self.room_types.copy()
            #     if "Corridor" in room_types:
            #         room_types.remove("Corridor")
            #     for i, room_type in enumerate(room_types):
            #         room_type_prompt += f"{i}. {room_type}\n"
            room_type_prompt = self.ROOM_TYPE_PROMPT_FREE

            # The object inventory is the primary signal; images are support.
            # if msg.objects:
            #     room_type_prompt += f"\nObjects detected in the room: {msg.objects}\n"

            # Label stability: keep the existing label unless the evidence clearly
            # indicates a different room type, and never swap it for a synonym
            # (e.g. do not relabel "meeting room" as "conference room").
            if msg.room_type:
                room_type_prompt += (
                    f"\nThis room is currently labeled \"{msg.room_type}\". "
                    "Return this exact same label unless the images and objects clearly "
                    "indicate a different kind of room. Do not change it to a synonym or "
                    "near-synonym of the current label.\n"
                )

            # Up to 3 room images followed by the top-down room-shape mask.
            content = [
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}}
                for b64 in image_b64_list
            ]
            if room_mask_base64 is not None:
                content.append(
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{room_mask_base64}"}})

            completion = self.vlm_model.beta.chat.completions.parse(
                model=self.room_type_vlm_model, # Use the flash lite model for faster response
                messages=[{
                    "role": "system",
                    "content": room_type_prompt
                }, {
                    "role": "user",
                    "content": content,
                }],
                response_format=Result,
            )
            # transform the image data to a format suitable for VLM
            try:
                answer = completion.choices[0].message.parsed
            except Exception:
                raw_text = completion.choices[0].message.content.strip()
                # 如果只是普通字符串，就包一层 JSON
                answer = Result(room_type=raw_text)
            # print the answer
            self.get_logger().info(f"Received room type answer: {answer}")
            room_type = answer.room_type
            self.get_logger().info(f"Determined room type: {room_type}")
            # Publish the room type answer
            answer_msg = msg
            answer_msg.room_type = room_type.lower()
            self.room_type_publisher.publish(answer_msg)
            # self.get_logger().info("Published room type answer")
            end_time = time.time()
            self.get_logger().info(f"Room type query processed in {end_time - start_time:.2f} seconds")

            # The room images already live on disk (planner room_views/); just
            # dump the mask for debugging if present.
            if cv_room_mask is not None:
                cv2.imwrite(f"debug/room_type/{msg.room_id}_{room_type}_mask.jpg", cv_room_mask)
            # save the answer to a text file for debugging
            answer_file_path = f"debug/room_type/{msg.room_id}_{room_type}.txt"
            with open(answer_file_path, 'w') as f:
                f.write(f"Room ID: {msg.room_id}\nIn Room: {msg.in_room}\nAnswer: {answer}\n")
        
        except Exception as e:
            self.get_logger().error(f"Error processing room type query: {e}")
    
    def process_object_type_query(self, msg: ObjectType):
        """Handle target object query and publish answer"""
        img = np.load(msg.img_path)
        mask_path = msg.img_path.replace('.npy', '_mask.npy')
        if os.path.exists(mask_path):
            mask = np.load(mask_path, mmap_mode=None)
            mask = cv2.dilate(mask.astype(np.uint8), np.ones((3, 3), np.uint8), iterations=3)
            # img = cv2.bitwise_and(img, img, mask=mask)

            # find contours of the mask and draw them
            mask_uint8 = mask.astype(np.uint8)
            contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # draw the contours on the image for debugging
            cv2.drawContours(img, contours, -1, (0, 255, 0), 2)
        img_jpg = cv2.imencode('.jpg', img)[1]
        img_base64 = base64.b64encode(img_jpg).decode('utf-8')
        labels = msg.labels
        class Result(BaseModel):
            reason: str
            label: str
        try:
            # Process the target object query
            completion = self.vlm_model.beta.chat.completions.parse(
                model=self.object_type_vlm_model,
                messages=[{
                    "role": "system",
                    "content": self.object_type_query_prompt
                }, {
                    "role":
                        "user",
                    "content": [
                        {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img_base64}"}},
                        {"type": "text", "text": f"Possible labels: {', '.join(self.object_list)}"},
                    ]
                }],
                response_format=Result,
            )
            answer = completion.choices[0].message.parsed
            # print the answer
            self.get_logger().info(f"Received target object answer: {answer}")
            verified_label = answer.label
            self.get_logger().info(f"Verified target object label: {verified_label}")
            # Publish the target object answer
            answer_msg = ObjectType()
            answer_msg.object_id = msg.object_id
            answer_msg.img_path = msg.img_path
            answer_msg.final_label = verified_label.lower()
            answer_msg.labels = msg.labels
            self.object_type_answer_publisher.publish(answer_msg)
            self.get_logger().info("Published target object answer")

            text = f"Object ID: {msg.object_id}\nVerified Label: {verified_label}\nPossible Labels: {', '.join(labels)}"
            # self.publish_text_overlay(text)

            # save the image for debugging
            img_path = f"debug/object_type/{msg.object_id}_{verified_label}.jpg"
            cv2.imwrite(img_path, img)
            # save the answer to a text file for debugging
            answer_file_path = f"debug/object_type/{msg.object_id}_{verified_label}.txt"
            with open(answer_file_path, 'w') as f:
                f.write(f"Verified Label: {verified_label}\nReason: {answer.reason}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing target object query: {e}")
    
    def vlm_node_callback(self):
        """Main loop to process queries"""
        # check if there are any room type queries
        if self.room_type_query_queue:
            latest_queries = {}
            # for each room_id, only keep the latest query
            while self.room_type_query_queue:
                item = self.room_type_query_queue.pop()  # 先出最新的
                room_id = item.room_id
                if room_id not in latest_queries or Time.from_msg(item.header.stamp) > Time.from_msg(latest_queries[room_id].header.stamp):
                    latest_queries[room_id] = item
            # using multithreading to process room type queries
            for room_id, query in latest_queries.items():
                self.get_logger().info(f"Processing room type query for room {room_id}")
                threading.Thread(target=self.process_room_type_query, args=(query,)).start()
        
        
        # check if there are any target object queries
        if self.object_type_query_queue:
            while self.object_type_query_queue:
                item = self.object_type_query_queue.popleft()
                self.get_logger().info(f"Processing target object query for object ID: {item.object_id}")
                threading.Thread(target=self.process_object_type_query, args=(item,)).start()
    
    
        
        

def main(args=None):
    rclpy.init(args=args)
    try:
        node = VLMNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("⏹️ Keyboard interrupt received")
    except Exception as e:
        print(f"❌ Failed to start VLM Node: {e}")
    finally:
        rclpy.shutdown()


# if __name__ == '__main__':
#     main()
