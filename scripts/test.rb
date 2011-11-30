require 'orocos'
include Orocos

if !ARGV[0]
    STDERR.puts "usage: test.rb <device name>"
    exit 1
end

ENV['PKG_CONFIG_PATH'] = "#{File.expand_path("..", File.dirname(__FILE__))}/build:#{ENV['PKG_CONFIG_PATH']}"

Orocos.initialize

Orocos::Process.run 'test' do |p|
    driver = Orocos::TaskContext.get 'xsens'
    task = Orocos::TaskContext.get 'auv_control'
#    Orocos.log_all_ports

    driver.port = ARGV[0]
    driver.configure
    driver.start


    driver.orientation_samples.connect_to task.pose_samples
    writer = task.motion_commands.writer
    sample = writer.new_sample
    sample.x_speed = 0 
    sample.y_speed = 0
    sample.z = -1 
    task.thruster_control_matrix =  1,          1,          0,      0,          0,      0, 
     0,          0,          0,      1,          0.2,     0,
     0,          0,          1,      0,          0,      0.2,
     0,          0,          0,      -0.2,       1,      0, 
     0,          0,          0,      0,          0,      1, 
     0,          0,          0,      0,          0,      0 
   

    pid = task.controller_yaw
    pid.Ts = 0.05
    pid.K = 1
    task.controller_yaw = pid
    task.controller_pitch = pid
    task.controller_roll = pid
    task.controller_x = pid
    task.controller_y = pid
    task.controller_z = pid



#    task.configure
    task.start
    loop do 
        sample.timestamp = Time.new
        writer.write(sample)
        pp task.state
	sleep 0.5
    end
end

