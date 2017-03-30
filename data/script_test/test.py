import sys, os, re
from subprocess import Popen, PIPE

def process_bagfile(bagfile, debug=True):

    basename_match = re.match("^(\w+)\.bag$", bagfile)
    if not basename_match:
        print "Not a valid bagfile: {}".format(bagfile)
        return
    else:
        basename = basename_match.group(1)

    if not os.path.isfile(bagfile):
        print "File does not exist: {}".format(bagfile)
        return

    if debug:
        print "Processing bagfile: {}".format(bagfile)

    if debug:
        print "Making directory: ~/{}".format(basename)
    mkdir_process = Popen(['mkdir', '-p', basename])

    rosbag_info_process = Popen(['rosbag', 'info', bagfile], stdout = PIPE, stderr = PIPE)
    rosbag_info_process.wait()
    for line in rosbag_info_process.stdout.readlines():
        topic_name = re.match(".*\s([a-zA-Z0-9/\_]+)\s+[0-9]+\s+msgs", line)
        if topic_name:
            print topic_name.group(1)





def main():
    for input_file in sys.argv[1:]:
        process_bagfile(input_file)

if __name__ == "__main__": main()
