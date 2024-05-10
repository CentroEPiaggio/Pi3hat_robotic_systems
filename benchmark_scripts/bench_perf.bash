

kin_chain="jnt_per_kin_1"
cu_par="chain_num_4"
imu=('0' '1')
# imu=( '1' ) 
main_t=( '750000' '1000000' '1250000' '1500000' '1750000')
# main_t=( '1250000' '1500000' '1750000' )
sec_t=('50000' '75000' '100000')


for use_imu in ${imu[@]}
do
    for m_t in ${main_t[@]}
    do
        for s_t in ${sec_t[@]}
        do
            bag_name="main_$m_t""_sec_$s_t""_use_imu_$use_imu"
            echo $bag_name
        
            ros2 launch pi3hat_hw_interface perform_bechmark.launch.py use_imu:=$use_imu duration_s:=300 kin_chain:=$kin_chain cu_param:=$cu_par bag_name:=$bag_name main_t:=$m_t can_t:=$s_t rec_t:=$s_t 
        done
    done
    
done
