import rosbag
import cv2
import numpy as np
import os
from tqdm import tqdm
from sensor_msgs.msg import JointState, Image, CompressedImage
import argparse
import logging
from datetime import datetime
import shutil

def setup_logging(output_dir):
    """配置日志记录"""
    os.makedirs(output_dir, exist_ok=True)
    log_file = os.path.join(output_dir, f"parse_bag_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(log_file),
            logging.StreamHandler()
        ]
    )
    return logging.getLogger('bag_parser')

def parse_bag(bag_file, output_dir):
    """解析单个bag文件"""
    logger = setup_logging(output_dir)
    logger.info(f"开始解析ROS bag文件: {bag_file}")
    
    # 创建输出子目录
    os.makedirs(os.path.join(output_dir, "images"), exist_ok=True)
    
    # 创建CSV文件记录所有消息
    all_msg_csv = os.path.join(output_dir, "all_messages.csv")
    with open(all_msg_csv, "w") as f:
        f.write("timestamp,topic,msg_type,data_summary\n")
    logger.info(f"创建消息记录文件: {all_msg_csv}")

    try:
        # 关键修改：允许未索引的bag文件
        with rosbag.Bag(bag_file, "r", allow_unindexed=True) as bag:
            # 获取总消息数用于进度条
            total_msgs = bag.get_message_count()
            logger.info(f"Bag文件统计: 总消息数={total_msgs}, 话题列表={list(bag.get_type_and_topic_info()[1].keys())}")
            
            for topic, msg, t in tqdm(bag.read_messages(), total=total_msgs, desc="解析进度"):
                timestamp = t.to_sec()
                msg_type = msg._type
                
                # 记录所有消息到CSV
                with open(all_msg_csv, "a") as f:
                    f.write(f"{timestamp:.6f},{topic},{msg_type},{str(msg)[:100]}...\n")
                
                # 特殊消息处理
                if msg_type == "sensor_msgs/JointState":
                    process_joint_state(topic, msg, timestamp, output_dir, logger)
                
                elif msg_type == "sensor_msgs/Image":
                    process_image(topic, msg, timestamp, output_dir, logger)
                
                elif msg_type == "sensor_msgs/CompressedImage":
                    process_compressed_image(topic, msg, timestamp, output_dir, logger)
                
                else:
                    logger.debug(f"忽略未处理的消息类型: {msg_type} (话题: {topic})")

    except Exception as e:
        logger.error(f"解析bag文件时发生错误: {str(e)}", exc_info=True)
        raise

    logger.info(f"解析完成！结果保存在: {output_dir}")

def process_joint_state(topic, msg, timestamp, output_dir, logger):
    """处理关节状态消息"""
    try:
        # 创建主题特定的输出目录
        topic_dir = os.path.join(output_dir, topic.replace('/', '_'))
        os.makedirs(topic_dir, exist_ok=True)
        
        joint_csv = os.path.join(topic_dir, "joint_states.csv")
        if not os.path.exists(joint_csv):
            with open(joint_csv, "w") as f:
                f.write("timestamp," + ",".join(msg.name) + "\n")
            logger.info(f"创建关节状态记录文件: {joint_csv}")
        
        with open(joint_csv, "a") as f:
            positions = ",".join(map(str, msg.position))
            f.write(f"{timestamp:.6f},{positions}\n")
        logger.debug(f"记录关节状态: {topic} @ {timestamp:.6f}")
    except Exception as e:
        logger.error(f"处理关节状态失败: {str(e)}", exc_info=True)

def process_image(topic, msg, timestamp, output_dir, logger):
    """处理普通图像消息"""
    try:
        logger.info(f"处理Image消息: topic={topic}, encoding={msg.encoding}, resolution={msg.width}x{msg.height}")
        
        # 创建主题特定的输出目录
        topic_dir = os.path.join(output_dir, "images", topic.replace('/', '_'))
        os.makedirs(topic_dir, exist_ok=True)
        
        if msg.encoding in ["rgb8", "bgr8", "mono8"]:
            cv_img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            if msg.encoding == "rgb8":
                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == "mono8":
                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
        elif msg.encoding == "16UC1":  # 深度图
            cv_img = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width)
            # 归一化以便显示
            cv_img = cv2.normalize(cv_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
        else:
            logger.warning(f"不支持的Image编码格式: {msg.encoding}")
            return
        
        img_path = os.path.join(topic_dir, f"{timestamp:.6f}.png")
        success = cv2.imwrite(img_path, cv_img)
        if success:
            logger.info(f"保存图像成功: {img_path}")
        else:
            logger.error(f"保存图像失败: {img_path}")
    except Exception as e:
        logger.error(f"处理Image消息失败: {str(e)}", exc_info=True)

def process_compressed_image(topic, msg, timestamp, output_dir, logger):
    """处理压缩图像消息"""
    try:
        logger.info(f"处理CompressedImage消息: topic={topic}, format={msg.format}")
        
        # 创建主题特定的输出目录
        topic_dir = os.path.join(output_dir, "images", topic.replace('/', '_'))
        os.makedirs(topic_dir, exist_ok=True)
        
        cv_img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if cv_img is None:
            logger.error("解压缩失败: 数据可能损坏")
            return
        
        img_path = os.path.join(topic_dir, f"{timestamp:.6f}.jpg")
        success = cv2.imwrite(img_path, cv_img)
        if success:
            logger.debug(f"保存压缩图像成功: {img_path}")
        else:
            logger.error(f"保存压缩图像失败: {img_path}")
    except Exception as e:
        logger.error(f"处理CompressedImage消息失败: {str(e)}", exc_info=True)

def parse_all_bags_in_folder(bag_folder, base_output_dir="output"):
    """解析文件夹中的所有bag文件"""
    if not os.path.exists(bag_folder):
        print(f"错误: 文件夹不存在 '{bag_folder}'")
        return
    
    if not os.path.isdir(bag_folder):
        print(f"错误: '{bag_folder}' 不是文件夹")
        return
    
    # 获取所有bag文件
    bag_files = [f for f in os.listdir(bag_folder) if f.endswith('.bag')]
    
    if not bag_files:
        print(f"警告: 文件夹 '{bag_folder}' 中没有找到 .bag 文件")
        return
    
    print(f"在文件夹 '{bag_folder}' 中找到 {len(bag_files)} 个bag文件:")
    for i, bag_file in enumerate(bag_files, 1):
        print(f"  {i}. {bag_file}")
    
    # 创建基础输出目录
    os.makedirs(base_output_dir, exist_ok=True)
    
    # 解析每个bag文件
    for bag_file in bag_files:
        bag_path = os.path.join(bag_folder, bag_file)
        print(f"\n{'='*50}")
        print(f"开始解析文件: {bag_file}")
        
        # 为每个bag文件创建独立的输出目录
        bag_name = os.path.splitext(bag_file)[0]
        output_dir = os.path.join(base_output_dir, bag_name)
        
        # 如果目录已存在，删除重建
        if os.path.exists(output_dir):
            shutil.rmtree(output_dir)
        os.makedirs(output_dir)
        
        print(f"输出目录: {output_dir}")
        
        try:
            parse_bag(bag_path, output_dir)
            print(f"解析完成！结果保存在: {output_dir}")
        except Exception as e:
            import traceback
            traceback.print_exc()
            print(f"解析失败: {str(e)}")
    
    print(f"\n{'='*50}")
    print(f"所有文件解析完成！结果保存在: {os.path.abspath(base_output_dir)}")

if __name__ == "__main__":
    # 设置要解析的文件夹路径
    bag_folder = "/home/robot/fleximind-ros1/data/bags"  # 修改为你的文件夹路径
    
    # 设置基础输出目录
    base_output_dir = "output"
    
    # 执行批量解析
    parse_all_bags_in_folder(bag_folder, base_output_dir)

