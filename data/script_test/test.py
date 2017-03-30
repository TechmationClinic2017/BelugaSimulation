import sys, os, re
from subprocess import Popen, PIPE
import numpy as np

def process_bagfile(bagfile, debug=True):
    
    # Validate the input bagfile
    basename_match = re.match("^(\w+)\.bag$", bagfile)
    
    if not basename_match:
        print "Not a valid bagfile: {}".format(bagfile)
        print
        return
    else:
        basename = basename_match.group(1)

    if not os.path.isfile(bagfile):
        print "File does not exist: {}".format(bagfile)
        print
        return

    if debug:
        print "Processing bagfile: {}".format(bagfile)

    # make a new directory to save all of the produced files
    if debug:
        print "Making directory: ~/{}".format(basename)

    mkdir_process = Popen(['mkdir', '-p', basename])
    mkdir_process.wait()

    # get all published topic names
    rosbag_info_process = Popen(['rosbag', 'info', bagfile], stdout = PIPE, stderr = PIPE)
    rosbag_info_process.wait()

    topic_names = []
    for line in rosbag_info_process.stdout.readlines():
        topic_name_match = re.match(".*\s([a-zA-Z0-9/\_]+)\s+[0-9]+\s+msgs", line)
        if topic_name_match:
            topic_names += [topic_name_match.group(1)]

    # extract data for each topic in the bagfile
    topic_dir = basename+"/topics/"
    mkdir_process = Popen(['mkdir', '-p', topic_dir])
    mkdir_process.wait()
    if debug:
        print "Processing topics and saving data to ~/{}:".format(topic_dir)

    for topic in topic_names:
        if debug:
            print "\t{}".format(topic)
        stripped_leader = re.sub('^/','',topic)
        sanitised_topicname = re.sub('/','-',stripped_leader)
        topic_filename = topic_dir + sanitised_topicname + ".dat"
        if os.path.isfile(topic_filename):
            if debug:
                print "\t\tSKIPPED: File already exists"
        else:
            topic_file = open(topic_filename,"w")
            save_topic_process = Popen(['rostopic', 'echo', '-p', '-b', bagfile, topic], stdout = topic_file)
            save_topic_process.wait()
            topic_file.close()

    # consolidate useful data
    data_dir = basename+"/data/"
    mkdir_process = Popen(['mkdir', '-p', data_dir])
    mkdir_process.wait()
    print "Extracting data and saving to ~/{}".format(data_dir)

    # get the test start time
    t_0 = float('inf')
    for control_input in ["right", "left", "top", "bottom"]:
        control_topic = "/beluga/control_inputs/" + control_input
        if control_topic in topic_names:
            control_topic_file = topic_dir + "beluga-control_inputs-" + control_input + ".dat"
            first_input = getFirstInputTime(control_topic_file)
            t_0 = min(t_0, first_input)

    if t_0 == float('inf'):
        t_0 = 0

    # save input data
    for control_input in ["right", "left", "top", "bottom"]:
        control_topic = "/beluga/control_inputs/" + control_input
        if control_topic not in topic_names:
            if debug:
                print "\tWARNING: No {} input data".format(control_input)
        else:
            control_topic_file = topic_dir + "beluga-control_inputs-" + control_input + ".dat"
            control_data_file = data_dir + control_input +"_input.dat"
            if debug:
                print "\tSaving {} input data as ~/{}".format(control_input, control_data_file)
            saveInputData(control_topic_file, control_data_file, start_time = t_0)

    ekf_topic = "/beluga/navigation/odometry/filtered"
    if ekf_topic not in topic_names:
        if debug:
            print "\tWARNING: No odometry data"
    else:
        ekf_topic_file = topic_dir+"/beluga-navigation-odometry-filtered.dat"
        ekf_data_file = data_dir + "ekf.dat"
        if debug:
            print "\tSaving odometry data as ~/{}".format(ekf_data_file)
        saveEkfData(ekf_topic_file, ekf_data_file, start_time = t_0)

    # plot the data and save graphs
    plot_dir = basename+"/plots/"
    mkdir_process = Popen(['mkdir', '-p', plot_dir])
    mkdir_process.wait()

def getFirstInputTime(control_topic_file):
    control_data = np.loadtxt(control_topic_file, delimiter=",", comments="%")
    first_ind = np.nonzero(control_data[:, 1])[0][0]
    return control_data[first_ind,0]

def saveInputData(control_topic_file, control_data_file, start_time = 0):
    control_data = np.loadtxt(control_topic_file, delimiter=",", comments="%")

    #scale the time data and subtract the start time
    control_data[:, 0] -= start_time
    control_data[:, 0] /= 1e9

    header = "Time\tInput"

    np.savetxt(control_data_file, control_data, fmt = "%.7f", delimiter = "\t", header = header)

    
def saveEkfData(ekf_topic_file, ekf_data_file, start_time = 0):
    ekf_data = np.loadtxt(ekf_topic_file, dtype=str, delimiter=",")

    save_indices = []
    header = ""

    t_ind = np.where(ekf_data[0] == "%time")[0][0]
    save_indices += [t_ind]
    header += "Time\t"

    x_ind = np.where(ekf_data[0] == "field.pose.pose.position.x")[0][0]
    y_ind = np.where(ekf_data[0] == "field.pose.pose.position.y")[0][0]
    z_ind = np.where(ekf_data[0] == "field.pose.pose.position.z")[0][0]
    save_indices += [x_ind, y_ind, z_ind]
    header += "x\ty\tz\t"

    qx_ind = np.where(ekf_data[0] == "field.pose.pose.orientation.x")[0][0]
    qy_ind = np.where(ekf_data[0] == "field.pose.pose.orientation.y")[0][0]
    qz_ind = np.where(ekf_data[0] == "field.pose.pose.orientation.z")[0][0]
    qw_ind = np.where(ekf_data[0] == "field.pose.pose.orientation.w")[0][0]
    save_indices += [qx_ind, qy_ind, qz_ind, qw_ind]
    header += "qx\tqy\tqz\tqw\t"

    vx_ind = np.where(ekf_data[0] == "field.twist.twist.linear.x")[0][0]
    vy_ind = np.where(ekf_data[0] == "field.twist.twist.linear.y")[0][0]
    vz_ind = np.where(ekf_data[0] == "field.twist.twist.linear.z")[0][0]
    save_indices += [vx_ind, vy_ind, vz_ind]
    header += "vx\tvy\tvz\t"

    data_to_save = ekf_data[1:, save_indices].astype(float)

    #scale the time data and subtract the start time
    data_to_save[:, 0] -= start_time
    data_to_save[:, 0] /= 1e9


    np.savetxt(ekf_data_file, data_to_save, fmt = "%.7f", delimiter = "\t", header = header)
    
    return


def generate_plots(data_dir, plot_dir):
    ekf_data = np.loadtxt(data_dir+"/beluga-navigation-odometry-filtered.dat", dtype=type(""), delimiter=",")

    t_ind = np.where(ekf_data[0] == "%time")[0][0]

    x_ind = np.where(ekf_data[0] == "field.pose.pose.position.x")[0][0]
    y_ind = np.where(ekf_data[0] == "field.pose.pose.position.y")[0][0]
    z_ind = np.where(ekf_data[0] == "field.pose.pose.position.z")[0][0]

    qx_ind = np.where(ekf_data[0] == "field.pose.pose.orientation.x")[0][0]
    qy_ind = np.where(ekf_data[0] == "field.pose.pose.orientation.y")[0][0]
    qz_ind = np.where(ekf_data[0] == "field.pose.pose.orientation.z")[0][0]
    qw_ind = np.where(ekf_data[0] == "field.pose.pose.orientation.w")[0][0]

    vx_ind = np.where(ekf_data[0] == "field.twist.twist.linear.x")[0][0]
    vy_ind = np.where(ekf_data[0] == "field.twist.twist.linear.y")[0][0]
    vz_ind = np.where(ekf_data[0] == "field.twist.twist.linear.z")[0][0]


def main():
    for input_file in sys.argv[1:]:
        process_bagfile(input_file)

if __name__ == "__main__": main()
