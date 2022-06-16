"""LFR_controller controller...
Project02"""


from controller import Robot

time_step=32
max_speed=-4

robot = Robot()


#motor
left_F_motor=robot.getDevice('wheel1')
right_F_motor=robot.getDevice('wheel2')
left_B_motor=robot.getDevice('wheel3')
right_B_motor=robot.getDevice('wheel4')

left_F_motor.setPosition(float('inf'))
right_F_motor.setPosition(float('inf'))
left_B_motor.setPosition(float('inf'))
right_B_motor.setPosition(float('inf'))

left_F_motor.setVelocity(0.0)
right_F_motor.setVelocity(0.0)
left_B_motor.setVelocity(0.0)
right_B_motor.setVelocity(0.0)


#ir sensor
right_ir=robot.getDevice('ds_right')
right_ir.enable(time_step)

mid_ir=robot.getDevice('ds_mid')
mid_ir.enable(time_step)

left_ir=robot.getDevice('ds_left')
left_ir.enable(time_step)


kp=0.6
ki=0.2
kd=0.6



base_speed=3

left_F_motor.setVelocity(base_speed)
right_F_motor.setVelocity(base_speed)
left_B_motor.setVelocity(base_speed)
right_B_motor.setVelocity(base_speed)

last_position=0


right_speed=base_speed
left_speed=base_speed

while robot.step(time_step) != -1:
   
   
   right_ir_val=right_ir.getValue()
   mid_ir_val=mid_ir.getValue()
   left_ir_val=left_ir.getValue()
   
   threshold_value=750
   
   if right_ir_val<threshold_value:
       right_ir_dg=0
   else:
       right_ir_dg=1

   if left_ir_val<threshold_value:
       left_ir_dg=0
   else:
       left_ir_dg=1
       
   if mid_ir_val<threshold_value:
       mid_ir_dg=0
   else:
       mid_ir_dg=1    
   
   weight=(-10)*left_ir_dg + 0*mid_ir_dg + 10*right_ir_dg
   
   sum= left_ir_dg+right_ir_dg+mid_ir_dg
   
   
   if sum==0:
       position=last_position
   else:
       position=weight/sum
       last_position=position
   print("left: {} mid: {} right: {}".format(left_ir_val,mid_ir_val,right_ir_val))
   print("sum:{} weight:{}".format(sum,weight))
   print("pos:{} last_pos:{}".format(position,last_position))
   
   motorCorrection=kp*position+ki*position+kd*(position-last_position) 
 
 
   right_speed=base_speed-motorCorrection
   left_speed=base_speed+motorCorrection  
 
   print("Left Speed:{} Right Speed:{}".format(left_speed,right_speed))  
  
   left_F_motor.setVelocity(left_speed)
   right_F_motor.setVelocity(right_speed)
   left_B_motor.setVelocity(left_speed)
   right_B_motor.setVelocity(right_speed) 
   pass