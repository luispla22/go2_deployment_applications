import argparse
from pathlib import Path
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import yaml
from collections import defaultdict
import time

def list_topics(reader):
    topics = set(conn.topic for conn in reader.connections)
    print("Topics in the bag:")
    for topic in sorted(topics):
        print(f"  {topic}")

def bag_info(reader):
    topics = defaultdict(lambda: {'msg_count': 0, 'type': None})
    duration = (reader.duration / 1e9)
    
    for conn, timestamp, _ in reader.messages():
        topics[conn.topic]['msg_count'] += 1
        topics[conn.topic]['type'] = conn.msgtype

    print(yaml.dump({
        'files': [str(p) for p in reader.paths],
        'duration': f"{duration:.2f}s",
        'start': reader.start_time,
        'end': reader.end_time,
        'messages': reader.message_count,
        'size': sum(p.stat().st_size for p in reader.paths),
        'topics': [{
            'topic': topic,
            'type': info['type'],
            'messages': info['msg_count'],
            'frequency': f"{info['msg_count']/duration:.2f}" if duration > 0 else "0"
        } for topic, info in topics.items()]
    }, sort_keys=False))

def echo_messages(reader, topics, num_messages):
    count = defaultdict(int)
    for conn, timestamp, raw_data in reader.messages():
        if not topics or conn.topic in topics:
            msg = deserialize_cdr(raw_data, conn.msgtype)
            print(f"\nTOPIC: {conn.topic}")
            print(f"TIMESTAMP: {timestamp}")
            print(f"MESSAGE:\n{msg}")
            
            count[conn.topic] += 1
            if num_messages and count[conn.topic] >= num_messages:
                if all(count[t] >= num_messages for t in topics):
                    break

def main():
    parser = argparse.ArgumentParser(description="ROS 2 bag inspection tool")
    parser.add_argument("bag_path", type=str, help="Path to the ROS 2 bag file or directory")
    parser.add_argument("command", choices=['list', 'info', 'echo'], help="Command to execute")
    parser.add_argument("-t", "--topics", nargs='+', help="Topics to echo (for echo command)")
    parser.add_argument("-n", "--num-messages", type=int, help="Number of messages to echo per topic (for echo command)")
    args = parser.parse_args()

    bag_path = Path(args.bag_path)
    if not bag_path.exists():
        print(f"Error: Bag file or directory not found at {bag_path}")
        return

    with Reader(bag_path) as reader:
        if args.command == 'list':
            list_topics(reader)
        elif args.command == 'info':
            bag_info(reader)
        elif args.command == 'echo':
            echo_messages(reader, set(args.topics) if args.topics else None, args.num_messages)

if __name__ == "__main__":
    main()