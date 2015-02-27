data = read.csv("data/eye_level_avg.csv")
head_data = subset(data, modality=="1")
arm_data = subset(data, modality=="4")
both_data = subset(data, modality=="5")

#head_y (screen horizontal)
head_desired_y = head_data$target_desired_y
head_error_y = head_data$target_head_actual_error_y
pdf("~/Desktop/scripts/figs/head_y.pdf")
plot(head_desired_y,head_error_y, type="o", col="black", xlim=c(-80,80), ylim=c(-30,30))
 points(head_desired_y,abs(head_error_y), type="o", col="blue")
title("Error in head_y (screen horizontal)")
legend("bottomleft", inset=.05, c("Raw","Absolute Value"), text.col=c("black","blue")) 
dev.off()

#head_z (screen vertical)
head_desired_z = head_data$target_desired_z
head_error_z = head_data$target_head_actual_error_z
pdf("~/Desktop/scripts/figs/head_z.pdf")
plot(head_desired_z,head_error_z, type="o", xlim=c(45,55), ylim=c(-10,10))
 points(head_desired_z,abs(head_error_z), type="o", col="blue")
title("Error in head_z (screen vertical)")
legend("bottomleft", inset=.05, c("Raw","Absolute Value"), text.col=c("black","blue")) 
dev.off()


#arm_y (screen horizontal)
arm_desired_y = arm_data$target_desired_y
arm_error_y = arm_data$target_arms_actual_y-arm_data$target_desired_y
("~/Desktop/scripts/figs/arm_y.pdf")
plot(arm_desired_y,arm_error_y, type="o", xlim=c(-80,80), ylim=c(-5,5))
 points(arm_desired_y,abs(arm_error_y), type="o", col="blue")
title(main="Error in arm_y (screen horizontal)",
		xlab="Desired",ylab="Error (inches)")
legend("bottomleft", inset=.05, c("Raw","Absolute Value"), text.col=c("black","blue")) 
dev.off()

#arm_z (screen vertical)
arm_desired_z = arm_data$target_desired_z
arm_error_z = arm_data$target_arms_actual_z-arm_data$target_desired_z
pdf("~/Desktop/scripts/figs/arm_z.pdf")
plot(arm_desired_z,arm_error_z, type="o", xlim=c(45,55), ylim=c(-10,10))
 points(arm_desired_z,abs(arm_error_z), type="o", col="blue")
title("Error in arm_z (screen vertical)")
legend("bottomleft", inset=.05, c("Raw","Absolute Value"), text.col=c("black","blue")) 
dev.off()


##both_y (screen horizontal)
#desired_arm_y = arm_data$target_desired_y
#error_arm_y = arm_data$target_arm_actual_y-arm_data$target_desired_y
#plot(desired_arm_y,error_arm_y)
