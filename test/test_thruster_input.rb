require 'minitest/spec'
require 'orocos/test/component'
require 'minitest/autorun'


describe 'auv_control::ThrustersInput configuration' do
  include Orocos::Test::Component
  start 'thrusters_input', 'auv_control::ThrustersInput' => 'thrusters_input'
  reader 'thrusters_input', 'cmd_out', :attr_name => 'cmd_out'
  writer 'thrusters_input', 'cmd_in', :attr_name => 'cmd_in'

  it 'thruster_coeff_pos < 0' do

    thrusters_input.apply_conf_file("auv_control::ThrustersInput.yml")

    aux = thrusters_input.thruster_coeff_pos
    aux[0] = -8
    thrusters_input.thruster_coeff_pos = aux
      
    assert_raises(Orocos::StateTransitionFailed) { thrusters_input.configure }
  end

  it 'thruster_coeff_neg < 0' do

    thrusters_input.apply_conf_file("auv_control::ThrustersInput.yml")

    aux = thrusters_input.thruster_coeff_neg
    aux[0] = -8
    thrusters_input.thruster_coeff_neg = aux
      
    assert_raises(Orocos::StateTransitionFailed) { thrusters_input.configure }
  end

  it 'wrong control_modes size' do

    thrusters_input.apply_conf_file("auv_control::ThrustersInput.yml",['default', 'control_mode_3_raw'], true)
    
    assert_raises(Orocos::StateTransitionFailed) { thrusters_input.configure }
  end

  it 'wrong thruster_coeff_pos size' do

    thrusters_input.apply_conf_file("auv_control::ThrustersInput.yml",['default', 'thruster_coeff_pos_size_3'], true)
    
    assert_raises(Orocos::StateTransitionFailed) { thrusters_input.configure }
  end

  it 'automatically setting control_modes' do

    thrusters_input.apply_conf_file("auv_control::ThrustersInput.yml")
    
    aux = thrusters_input.control_modes
    aux.clear
    thrusters_input.control_modes = aux 
      
    thrusters_input.configure
  end

  it 'pos and neg coefficients have different sizes' do

    thrusters_input.apply_conf_file("auv_control::ThrustersInput.yml",['default', 'thruster_coeff_pos_size_3'], true)
    
    aux = thrusters_input.control_modes
    aux.clear
    thrusters_input.control_modes = aux 
      
    assert_raises(Orocos::StateTransitionFailed) { thrusters_input.configure }
  end

  it 'control modes set to EFFORT' do

    thrusters_input.apply_conf_file("auv_control::ThrustersInput.yml",['default', 'control_modes_effort'], true)
    
    assert_raises(Orocos::StateTransitionFailed) { thrusters_input.configure }
  end

  it 'control mode equals to RAW but no thruster voltage was set' do

    thrusters_input.apply_conf_file("auv_control::ThrustersInput.yml")
    
    thrusters_input.thruster_voltage = 0
    
    assert_raises(Orocos::StateTransitionFailed) { thrusters_input.configure }
  end

  it 'wrong number of thrusters input' do

    thrusters_input.apply_conf_file("auv_control::ThrustersInput.yml")
    
    thrusters_input.configure  
    thrusters_input.start  
      
    sample = thrusters_input.cmd_in.new_sample
    
    # 3 thrusters inputs were sent and 2 are expected
    thruster = Types::Base::JointState.new
    thruster.effort = 3
    sample.elements = [thruster, thruster, thruster]
    cmd_in.write sample 

    assert_state_change(thrusters_input) { |s| s == :UNEXPECTED_THRUSTER_INPUT } 
        
  end
  
  it 'thruster input not set' do

    thrusters_input.apply_conf_file("auv_control::ThrustersInput.yml")
    
    thrusters_input.configure  
    thrusters_input.start  

    sample = thrusters_input.cmd_in.new_sample
      
    # the effort field was not set
    thruster = Types::Base::JointState.new
    sample.elements = [thruster, thruster]
    cmd_in.write sample 

    assert_state_change(thrusters_input) { |s| s == :UNSET_THRUSTER_INPUT } 
        
  end
  
  it 'testing positive speed calculated value' do

    thrusters_input.apply_conf_file("auv_control::ThrustersInput.yml",['default', 'control_modes_speed'], true)
    
    thrusters_input.configure  
    thrusters_input.start  

    sample = thrusters_input.cmd_in.new_sample
      
    thruster1 = Types::Base::JointState.new
    thruster2 = Types::Base::JointState.new
    thruster1.effort = 37
    thruster2.effort = 43
    sample.elements = [thruster1, thruster2]
    cmd_in.write sample 

    data = assert_has_one_new_sample cmd_out, 1
    
    assert (data.elements[0].speed - 1.9235 < 0.001)
    assert (data.elements[1].speed - 2.07364 < 0.001)
        
  end

  it 'testing negative speed calculated value' do

    thrusters_input.apply_conf_file("auv_control::ThrustersInput.yml",['default', 'control_modes_speed'], true)
    
    thrusters_input.configure  
    thrusters_input.start  

    sample = thrusters_input.cmd_in.new_sample
      
    thruster1 = Types::Base::JointState.new
    thruster2 = Types::Base::JointState.new
    thruster1.effort = -15
    thruster2.effort = -7
    sample.elements = [thruster1, thruster2]
    cmd_in.write sample 

    data = assert_has_one_new_sample cmd_out, 1
    
    assert (data.elements[0].speed + 1.3693 < 0.001)
    assert (data.elements[1].speed + 0.9354 < 0.001)
        
  end

  it 'testing positive raw calculated value' do

    thrusters_input.apply_conf_file("auv_control::ThrustersInput.yml")
    
    thrusters_input.configure  
    thrusters_input.start  

    sample = thrusters_input.cmd_in.new_sample
      
    thruster1 = Types::Base::JointState.new
    thruster2 = Types::Base::JointState.new
    thruster1.effort = 17
    thruster2.effort = 3
    sample.elements = [thruster1, thruster2]
    cmd_in.write sample 

    data = assert_has_one_new_sample cmd_out, 1
    
    assert (data.elements[0].raw - 0.0686 < 0.001)
    assert (data.elements[1].raw - 0.0288 < 0.001)
  end

  it 'testing negative raw calculated value' do

    thrusters_input.apply_conf_file("auv_control::ThrustersInput.yml")
    
    thrusters_input.configure  
    thrusters_input.start  

    sample = thrusters_input.cmd_in.new_sample
      
    thruster1 = Types::Base::JointState.new
    thruster2 = Types::Base::JointState.new
    thruster1.effort = -49
    thruster2.effort = -31
    sample.elements = [thruster1, thruster2]
    cmd_in.write sample 

    data = assert_has_one_new_sample cmd_out, 1
    
    assert (data.elements[0].raw + 0.1302 < 0.001)
    assert (data.elements[1].raw + 0.1036 < 0.001)
  end

end

