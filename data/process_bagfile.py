import sys, os, re
from subprocess import Popen, PIPE
import numpy as np
import tf

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def process_bagfile(bagfile, debug=True):
    
    # Validate the input bagfile
    basename_match = re.match("^([\w-]+)\.bag$", bagfile)
    
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

    mkdir_process = Popen(["mkdir", "-p", basename])
    mkdir_process.wait()

    # get all published topic names
    rosbag_info_process = Popen(["rosbag", "info", bagfile], stdout = PIPE, stderr = PIPE)
    rosbag_info_process.wait()

    topic_names = []
    for line in rosbag_info_process.stdout.readlines():
        topic_name_match = re.match(".*\s([a-zA-Z0-9/\_]+)\s+[0-9]+\s+msgs", line)
        if topic_name_match:
            topic_names += [topic_name_match.group(1)]

    # extract data for each topic in the bagfile
    topic_dir = basename+"/topics/"
    mkdir_process = Popen(["mkdir", "-p", topic_dir])
    mkdir_process.wait()
    if debug:
        print "Processing topics and saving data to ~/{}:".format(topic_dir)

    for topic in topic_names:
        if debug:
            print "\t{}".format(topic)
        stripped_leader = re.sub("^/","",topic)
        sanitised_topicname = re.sub("/","-",stripped_leader)
        topic_filename = topic_dir + sanitised_topicname + ".dat"
        if os.path.isfile(topic_filename):
            if debug:
                print "\t\tSKIPPED: File already exists"
        else:
            topic_file = open(topic_filename,"w")
            save_topic_process = Popen(["rostopic", "echo", "-p", "-b", bagfile, topic], stdout = topic_file)
            save_topic_process.wait()
            topic_file.close()

    # consolidate useful data
    data_dir = basename+"/data/"
    mkdir_process = Popen(["mkdir", "-p", data_dir])
    mkdir_process.wait()
    print "Extracting data and saving to ~/{}".format(data_dir)

    # get the test start and end time
    t_0 = float("inf")
    t_f = -float("inf")
    for control_input in ["right", "left", "top", "bottom"]:
        control_topic = "/beluga/control_inputs/" + control_input
        if control_topic in topic_names:
            control_topic_file = topic_dir + "beluga-control_inputs-" + control_input + ".dat"
            first_input,last_input = getInputTime(control_topic_file)
            t_0 = min(t_0, first_input)
            t_f = max(t_f, last_input)

    if t_0 == float("inf"):
        t_0 = 0

    test_time = (t_f - t_0)/1e9
    if test_time < 0:
        test_time = float("inf")

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

    # save ekf data
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
    mkdir_process = Popen(["mkdir", "-p", plot_dir])
    mkdir_process.wait()
    generatePlots(data_dir, plot_dir, start_time = 0, end_time = test_time)

    if debug:
        print "Coppying original bagfile inside created directory"
    cp_process = Popen(["cp", bagfile, basename])
    cp_process.wait()
    if debug:
        print "Bagfile successfully processed\n"


def getInputTime(control_topic_file):
    control_data = np.loadtxt(control_topic_file, delimiter=",", comments="%")
    nonzero_control_input = np.nonzero(control_data[:, 1])[0]
    if nonzero_control_input.size == 0:
        t_0 = float("inf")
        t_f = -float("inf")
    else:
        first_ind = nonzero_control_input[0]
        last_ind  = nonzero_control_input[-1]
        t_0 = control_data[first_ind,0]
        t_f = control_data[last_ind,0]
    return t_0, t_f

def saveInputData(control_topic_file, control_data_file, start_time = 0):
    control_data = np.loadtxt(control_topic_file, delimiter=",", comments="%")

    #scale the time data and subtract the start time
    control_data[:, 0] -= start_time
    control_data[:, 0] /= 1e9

    header = "Time\tInput"

    np.savetxt(control_data_file, control_data, fmt = "%.7f", delimiter = "\t", header = header)

    
def saveEkfData(ekf_topic_file, ekf_data_file, start_time = 0):
    ekf_data = np.loadtxt(ekf_topic_file, dtype=str, delimiter=",")

    header = ""

    t_ind = np.where(ekf_data[0] == "%time")[0][0]
    data_to_save = ekf_data[1:,t_ind].astype(float)
    data_to_save = data_to_save.reshape(data_to_save.shape[0], -1)
    data_to_save[:, 0] -= start_time
    data_to_save[:, 0] /= 1e9
    header += "Time\t"

    x_ind = np.where(ekf_data[0] == "field.pose.pose.position.x")[0][0]
    y_ind = np.where(ekf_data[0] == "field.pose.pose.position.y")[0][0]
    z_ind = np.where(ekf_data[0] == "field.pose.pose.position.z")[0][0]
    pos_ind = [x_ind, y_ind, z_ind]
    data_to_save = np.append(data_to_save, ekf_data[1:,pos_ind].astype(float), axis = 1)
    header += "x\ty\tz\t"

    qx_ind = np.where(ekf_data[0] == "field.pose.pose.orientation.x")[0][0]
    qy_ind = np.where(ekf_data[0] == "field.pose.pose.orientation.y")[0][0]
    qz_ind = np.where(ekf_data[0] == "field.pose.pose.orientation.z")[0][0]
    qw_ind = np.where(ekf_data[0] == "field.pose.pose.orientation.w")[0][0]
    q_inds = [qx_ind, qy_ind, qz_ind, qw_ind]
    rpy = np.apply_along_axis(lambda q: tf.transformations.euler_from_quaternion(q), 1, ekf_data[1:, q_inds])
    data_to_save = np.append(data_to_save, rpy, axis = 1)
    header += "roll\tpitch\tyaw\t"

    vx_ind = np.where(ekf_data[0] == "field.twist.twist.linear.x")[0][0]
    vy_ind = np.where(ekf_data[0] == "field.twist.twist.linear.y")[0][0]
    vz_ind = np.where(ekf_data[0] == "field.twist.twist.linear.z")[0][0]
    vel_ind = [vx_ind, vy_ind, vz_ind]
    data_to_save = np.append(data_to_save, ekf_data[1:,vel_ind].astype(float), axis = 1)
    header += "vx\tvy\tvz\t"

    np.savetxt(ekf_data_file, data_to_save, fmt = "%.7f", delimiter = "\t", header = header)
    
    return


def generatePlots(data_dir, plot_dir, start_time = -float("inf"), end_time = float("inf"), debug = True):
    if debug:
        print "Generating plots and saving to ~/{}:".format(plot_dir)

    # NOTE: this function assumes that the IMU is in the incorrect position, and is reporting -vy for forward velocity
    # and the recorded roll is actually pitch

    # load data
    t,x,y,z,r,p,y,vx,vy,vz = np.loadtxt(data_dir+"ekf.dat", delimiter="\t", comments="#", unpack=True)
    ind = np.where( (start_time <= t)*(t <= end_time))
    
    # Forward velocity
    fig, ax = plt.subplots( nrows = 1, ncols = 1)
    ax.plot(t[ind],-vy[ind])
    ax.grid(True)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s)")
    ax.set_title("Forward Velocity vs. Time")
    vel_plot_file = plot_dir + "vel.png"
    fig.savefig(vel_plot_file)
    if debug:
        print "\tSaving forward velocity plot as ~/{}".format(vel_plot_file)

    # Pitch
    fig, ax = plt.subplots( nrows = 1, ncols = 1)
    ax.plot(t[ind],r[ind])
    ax.grid(True)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Pitch (rad)")
    ax.set_title("Pitch vs. Time")
    pitch_plot_file = plot_dir + "pitch.png"
    fig.savefig(pitch_plot_file)
    if debug:
        print "\tSaving pitch plot as ~/{}".format(pitch_plot_file)

    # Yaw
    fig, ax = plt.subplots( nrows = 1, ncols = 1)
    ax.plot(t[ind],y[ind])
    ax.grid(True)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Yaw (rad)")
    ax.set_title("Yaw vs. Time")
    yaw_plot_file = plot_dir + "yaw.png"
    fig.savefig(yaw_plot_file)
    if debug:
        print "\tSaving yaw plot as ~/{}".format(yaw_plot_file)

def main():
    for input_file in sys.argv[1:]:
        process_bagfile(input_file)

if __name__ == "__main__": main()
