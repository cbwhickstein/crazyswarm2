from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
import rosbag2_py
import csv
from numpy import NaN

class McapHandler:
    # def __init__(self, rosclock : bool):
    #     self.rosclock = rosclock

    def read_messages(self, input_bag: str):
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"
            ),
        )
        topic_types = reader.get_all_topics_and_types()

        def typename(topic_name):
            for topic_type in topic_types:
                if topic_type.name == topic_name:
                    return topic_type.type
            raise ValueError(f"topic {topic_name} not in bag")

        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            msg_type = get_message(typename(topic))
            msg = deserialize_message(data, msg_type)
            yield topic, msg, timestamp
        del reader
    
    def write_mcap_to_csv(self, inputbag:str, outputfile:str):
        '''A method which translates an .mcap rosbag file format to a .csv file. 
        Only written to translate the /tf topic but could easily be extended to other topics'''

        try:
            print("Translating .mcap to .csv")
            f = open(outputfile, 'w+')
            writer = csv.writer(f)
            writer.writerow(["#topic", "tf_timestamp","tf_x","tf_y","tf_z","clock_sec","clock_nanosec"])  
            #NB : when the message doesn't contain the corresponding field, NaN is written instead          
            for topic, msg, timestamp in self.read_messages(inputbag):
                    if topic == "/tf":
                        writer.writerow(["/tf", timestamp, msg.transforms[0].transform.translation.x, msg.transforms[0].transform.translation.y, msg.transforms[0].transform.translation.z, NaN, NaN])
                    elif topic == "/clock":
                        writer.writerow(["/clock",NaN,NaN,NaN,NaN, msg.clock.sec, msg.clock.nanosec])
                    else :
                        print(f"Mcaphandler : topic {topic} not supported")
                        exit(1)
            # for topic, msg, timestamp in self.read_messages(inputbag):
            #     writer.writerow([timestamp, msg.transforms[0].transform.translation.x, msg.transforms[0].transform.translation.y, msg.transforms[0].transform.translation.z])
            f.close()
        except FileNotFoundError:
            print(f"McapHandler : file {outputfile} not found")
            exit(1)




if __name__ == "__main__":

    translate = McapHandler()
    translate.write_mcap_to_csv("/home/julien/ros2_ws/src/crazyswarm2/systemtests/tfclockbag_0.mcap","/home/julien/ros2_ws/src/crazyswarm2/systemtests/figure8_tfclockbag_0.csv")

    #command line utility 

    # from argparse import ArgumentParser, Namespace
    # parser = ArgumentParser(description="Translates the /tf topic of an .mcap rosbag file format to a .csv file")
    # parser.add_argument("inputbag", type=str, help="The .mcap rosbag file to be translated")
    # parser.add_argument("outputfile", type=str, help="Output csv file that has to be created/overwritten")
    # parser.add_argument("--rosclock", action="store_true", help="Translate messages from the /clock (ROS simulation time) topic too")
    # args:Namespace = parser.parse_args()

    # translator =  McapHandler(args.rosclock)
    # translator.write_mcap_to_csv(args.inputbag,args.outputfile)
