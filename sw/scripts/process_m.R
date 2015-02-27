source("~/Desktop/lg_april/scripts/GetCalPts.R")
data = read.csv("all_data.csv")
attach(data)
data$target_desired_y = data$target_desired_y*2.54
data$target_desired_z = data$target_desired_z*2.54
data$target_head_actual_z = data$target_head_actual_z*2.54
data$target_head_actual_z = data$target_head_actual_z*2.54
data$target_arms_actual_z = data$target_arms_actual_z*2.54
data$target_arms_actual_z = data$target_arms_actual_z*2.54
data$target_perceived_z = data$target_perceived_z*2.54
data$target_perceived_z = data$target_perceived_z*2.54

head_data = subset(data, modality=="1")
arm_data = subset(data, modality=="4")
both_data = subset(data, modality=="5")
head_calib_near = subset(data, modality=="1" & target_desired_y==9*2.54)
arm_calib_near = subset(data, modality=="4" & target_desired_y==9*2.54)
cross_calib_near = subset(data, modality=="4" & target_desired_y==-9*2.54)
head_calib_far = subset(data, modality=="1" & target_desired_y==-27*2.54)
arm_calib_far = subset(data, modality=="4" & target_desired_y==27*2.54)
cross_calib_far = subset(data, modality=="4" & target_desired_y==-27*2.54)

detach()
attach(head_calib_near)
head_near_per_des_error = distance2(target_desired_y, target_perceived_y, target_desired_z, target_perceived_z)
head_near_per_des_error_y = target_desired_y-target_perceived_y
head_near_per_des_error_z = target_desired_z-target_perceived_z

detach()
attach(head_calib_far)
head_far_per_des_error = distance2(target_desired_y, target_perceived_y, target_desired_z, target_perceived_z)
head_far_per_des_error_y = target_desired_y-target_perceived_y
head_far_per_des_error_z = target_desired_z-target_perceived_z

detach()
attach(arm_calib_near)
arm_near_per_des_error = distance2(target_desired_y, target_perceived_y, target_desired_z, target_perceived_z)
arm_near_per_des_error_y = target_desired_y-target_perceived_y
arm_near_per_des_error_z = target_desired_z-target_perceived_z

detach()
attach(arm_calib_far)
arm_far_per_des_error = distance2(target_desired_y, target_perceived_y, target_desired_z, target_perceived_z)
arm_far_per_des_error_y = target_desired_y-target_perceived_y
arm_far_per_des_error_z = target_desired_z-target_perceived_z

detach()
attach(cross_calib_near)
cross_near_per_des_error = distance2(target_desired_y, target_perceived_y, target_desired_z, target_perceived_z)
cross_near_per_des_error_y = target_desired_y-target_perceived_y
cross_near_per_des_error_z = target_desired_z-target_perceived_z

detach()
attach(cross_calib_far)
cross_far_per_des_error = distance2(target_desired_y, target_perceived_y, target_desired_z, target_perceived_z)
cross_far_per_des_error_y = target_desired_y-target_perceived_y
cross_far_per_des_error_z = target_desired_z-target_perceived_z
detach()


pdf("~/Desktop/figs/head_near_per_des_error.pdf")
hist(head_near_per_des_error, xlim=c(0, 150))
line(density(head_near_per_des_error))
dev.off()

pdf("~/Desktop/figs/arm_near_per_des_error.pdf")
hist(arm_near_per_des_error, xlim=c(0, 150))
dev.off()

pdf("~/Desktop/figs/cross_near_per_des_error.pdf")
hist(cross_near_per_des_error, xlim=c(0, 150))
dev.off()

pdf("~/Desktop/figs/head_near_per_des_error_y.pdf")
hist(head_near_per_des_error_y, xlim=c(0, 150))
dev.off()

pdf("~/Desktop/figs/arm_near_per_des_error_y.pdf")
hist(arm_near_per_des_error_y, xlim=c(0, 150))
dev.off()

pdf("~/Desktop/figs/cross_near_per_des_error_y.pdf")
hist(cross_near_per_des_error_y, xlim=c(0, 150))
dev.off()

pdf("~/Desktop/figs/head_near_per_des_error_z.pdf")
hist(head_near_per_des_error_z, xlim=c(0, 150))
dev.off()

pdf("~/Desktop/figs/arm_near_per_des_error_z.pdf")
hist(arm_near_per_des_error_z, xlim=c(0, 150))
dev.off()

pdf("~/Desktop/figs/cross_near_per_des_error_z.pdf")
hist(cross_near_per_des_error_z, xlim=c(0, 150))
dev.off()

pdf("~/Desktop/figs/head_far_per_des_error_y.pdf")
hist(head_far_per_des_error_y, xlim=c(0, 150))
dev.off()

pdf("~/Desktop/figs/arm_far_per_des_error_y.pdf")
hist(arm_far_per_des_error_y, xlim=c(0, 150))
dev.off()

pdf("~/Desktop/figs/cross_far_per_des_error_y.pdf")
hist(cross_far_per_des_error_y, xlim=c(0, 150))
dev.off()

pdf("~/Desktop/figs/head_far_per_des_error_z.pdf")
hist(head_far_per_des_error_z, xlim=c(0, 150))
dev.off()

pdf("~/Desktop/figs/arm_far_per_des_error_z.pdf")
hist(arm_far_per_des_error_z, xlim=c(0, 150))
dev.off()

pdf("~/Desktop/figs/cross_far_per_des_error_z.pdf")
hist(cross_far_per_des_error_z, xlim=c(0, 150))
dev.off()

pdf("~/Desktop/figs/head_far_per_des_error.pdf")
hist(head_far_per_des_error, xlim=c(0, 150))
dev.off()

pdf("~/Desktop/figs/arm_far_per_des_error.pdf")
hist(arm_far_per_des_error, xlim=c(0, 150))
dev.off()

pdf("~/Desktop/figs/cross_far_per_des_error.pdf")
hist(cross_far_per_des_error, xlim=c(0, 150))
dev.off()

#density plots
pdf("~/Desktop/figs/densities.pdf")
par(mfrow=c(2,1))
plot(density(head_near_per_des_error, kernel="gaussian"), xlim=range(c(head_near_per_des_error, arm_near_per_des_error, cross_near_per_des_error)), ylim=c(0,0.3), col=2, xlab="Error (inches)", main="Error PDF, near calibration point, by modality")
lines(density(arm_near_per_des_error, kernel="gaussian"), col=3)
lines(density(cross_near_per_des_error, kernel="gaussian"), col=4)
legend(x="topright", c("head", "arm", "bent arm"), col=c(2,3,4), pch=15)
plot(density(head_far_per_des_error, kernel="gaussian"), xlim=range(c(head_near_per_des_error, arm_near_per_des_error, cross_near_per_des_error)), ylim=c(0,0.3), col=2, xlab="Error (inches)", main="Error PDF, far calibration point, by modality")
lines(density(arm_far_per_des_error, kernel="gaussian"), col=3)
lines(density(cross_far_per_des_error, kernel="gaussian"), col=4)
dev.off()

pdf("~/Desktop/figs/combined_densities_far.pdf")
par(mfrow=c(2,2))
plot(density(head_near_per_des_error_y, kernel="gaussian"), xlim=c(-100,100), ylim=c(0,0.3), col=2, xlab="Horizontal Error (inches)", main="Error PDF, near calibration point")
lines(density(arm_near_per_des_error_y, kernel="gaussian"), col=3)
lines(density(cross_near_per_des_error_y, kernel="gaussian"), col=4)
legend(x="topright", c("head", "arm", "bent arm"), col=c(2,3,4), pch=15)
plot(density(head_far_per_des_error_z, kernel="gaussian"), xlim=c(-100,100), ylim=c(0,0.3), col=2, xlab="Vertical Error (inches)", main="Error PDF, far calibration point")
lines(density(arm_far_per_des_error_z, kernel="gaussian"), col=3)
lines(density(cross_far_per_des_error_z, kernel="gaussian"), col=4)
legend(x="topright", c("head", "arm", "bent arm"), col=c(2,3,4), pch=15)
plot(density(head_far_per_des_error_y, kernel="gaussian"), xlim=c(-100,100), ylim=c(0,0.3), col=2, xlab="Horizontal Error (inches)", main="Error PDF, far calibration point")
lines(density(arm_far_per_des_error_y, kernel="gaussian"), col=3)
lines(density(cross_far_per_des_error_y, kernel="gaussian"), col=4)
legend(x="topright", c("head", "arm", "bent arm"), col=c(2,3,4), pch=15)
plot(density(head_near_per_des_error_z, kernel="gaussian"), xlim=c(-100,100), ylim=c(0,0.3), col=2, xlab="Vertical Error (inches)", main="Error PDF, near calibration point")
lines(density(arm_near_per_des_error_z, kernel="gaussian"), col=3)
lines(density(cross_near_per_des_error_z, kernel="gaussian"), col=4)
legend(x="topright", c("head", "arm", "bent arm"), col=c(2,3,4), pch=15)

dev.off()

pdf("~/Desktop/figs/densities_far.pdf")
plot(density(head_far_per_des_error, kernel="gaussian"), xlim=range(c(head_near_per_des_error, arm_near_per_des_error, cross_near_per_des_error)), ylim=c(0,0.3), col=2, xlab="Error (inches)", main="Error density far calibration point, by modality")
lines(density(arm_far_per_des_error, kernel="gaussian"), col=3)
lines(density(cross_far_per_des_error, kernel="gaussian"), col=4)
legend(x="topright", c("head", "arm", "bent arm"), col=c(2,3,4), pch=15)
dev.off()

pdf("~/Desktop/figs/densities_near.pdf")
plot(density(head_near_per_des_error, kernel="gaussian"), xlim=range(c(head_near_per_des_error, arm_near_per_des_error, cross_near_per_des_error)), ylim=c(0,0.3), col=2, xlab="Error (inches)", main="Error density near calibration point, by modality")
lines(density(arm_near_per_des_error, kernel="gaussian"), col=3)
lines(density(cross_near_per_des_error, kernel="gaussian"), col=4)
legend(x="topright", c("head", "arm", "bent arm"), col=c(2,3,4), pch=15)
dev.off()

pdf("~/Desktop/figs/densities_near_y.pdf")
plot(density(head_near_per_des_error_y, kernel="gaussian"), xlim=range(c(head_near_per_des_error_y, arm_near_per_des_error_y, cross_near_per_des_error_y)), ylim=c(0,0.3), col=2, xlab="Horizontal Error (inches)", main="Error density near calibration point, by modality")
lines(density(arm_near_per_des_error_y, kernel="gaussian"), col=3)
lines(density(cross_near_per_des_error_y, kernel="gaussian"), col=4)
legend(x="topright", c("head", "arm", "bent arm"), col=c(2,3,4), pch=15)
dev.off()

pdf("~/Desktop/figs/densities_near_z.pdf")
plot(density(head_near_per_des_error_z, kernel="gaussian"), xlim=range(c(head_near_per_des_error_z, arm_near_per_des_error_z, cross_near_per_des_error_z)), ylim=c(0,0.3), col=2, xlab="Vertical Error (inches)", main="Error density near calibration point, by modality")
lines(density(arm_near_per_des_error_z, kernel="gaussian"), col=3)
lines(density(cross_near_per_des_error_z, kernel="gaussian"), col=4)
legend(x="topright", c("head", "arm", "bent arm"), col=c(2,3,4), pch=15)
dev.off()

pdf("~/Desktop/figs/densities_near_y_z.pdf")
par(mfrow=c(2,1))
plot(density(head_near_per_des_error_y, kernel="gaussian"), xlim=range(c(head_near_per_des_error_y, arm_near_per_des_error_y, cross_near_per_des_error_y)), ylim=c(0,0.3), col=2, xlab="Horizontal Error (inches)", main="Error PDF, near calibration point, by modality")
lines(density(arm_near_per_des_error_y, kernel="gaussian"), col=3)
lines(density(cross_near_per_des_error_y, kernel="gaussian"), col=4)
legend(x="topright", c("head", "arm", "bent arm"), col=c(2,3,4), pch=15)

plot(density(head_near_per_des_error_z, kernel="gaussian"), xlim=range(c(head_near_per_des_error_z, arm_near_per_des_error_z, cross_near_per_des_error_z)), ylim=c(0,0.3), col=2, xlab="Vertical Error (inches)", main="Error PDF, near calibration point, by modality")
lines(density(arm_near_per_des_error_z, kernel="gaussian"), col=3)
lines(density(cross_near_per_des_error_z, kernel="gaussian"), col=4)
dev.off()

#qq plots
pdf("~/Desktop/figs/qq_norm_head_near.pdf")
qqnorm(head_near_per_des_error, ylim=c(0,30))
abline(0,1)
dev.off()
pdf("~/Desktop/figs/qq_norm_arm_near.pdf")
qqnorm(arm_near_per_des_error, ylim=c(0,30))
abline(0,1)
dev.off()
pdf("~/Desktop/figs/qq_norm_cross_near.pdf")
qqnorm(cross_near_per_des_error, ylim=c(0,30))
abline(0,1)
dev.off()


pdf("~/Desktop/figs/hist_near_combined.pdf")
par(mar=c(1,1,1,1), mfrow=c(1,3))
hist(head_far_per_des_error, xlim=c(0, 50), col=2)
hist(arm_far_per_des_error, xlim=c(0, 50), col=3)
hist(cross_far_per_des_error, xlim=c(0, 50), col=4)
dev.off()


#all points
pdf("~/Desktop/figs/target_agg.pdf")
plot(data$target_perceived_y, data$target_perceived_z, xlim=c(-80,80), ylim=c(0,100))
dev.off()
pdf("~/Desktop/figs/target_agg_per.pdf")
plot(data$target_perceived_y, data$target_perceived_z, xlim=c(-80,80), ylim=c(0,100))
dev.off()

pdf("~/Desktop/figs/target_agg_head_actual.pdf")
plot(head_data$target_head_actual_y, head_data$target_head_actual_z, xlim=c(-80,80), ylim=c(0,100), col="blue")
dev.off()
pdf("~/Desktop/figs/target_agg_arm_actual.pdf")
plot(arm_data$target_arms_actual_y, arm_data$target_arms_actual_z, xlim=c(-80,80), ylim=c(0,100), col="green")
dev.off()
pdf("~/Desktop/figs/target_agg_both_actual.pdf")
#plot((both_data$target_head_actual_y+both_data$target_arms_actual_y)/2,(both_data$target_head_actual_z+both_data$target_arms_actual_z)/2, xlim=c(-80,80), ylim=c(0,100))
plot(both_data$target_head_actual_y,both_data$target_head_actual_z, xlim=c(-80,80), ylim=c(0,100), col="orange", pch=1)
points(both_data$target_head_actual_y,both_data$target_head_actual_z, xlim=c(-80,80), ylim=c(0,100), col="orange", pch=4)
dev.off()

pdf("~/Desktop/figs/target_agg_all_actual.pdf")
plot(head_data$target_head_actual_y, head_data$target_head_actual_z, xlim=c(-80,80), ylim=c(0,100), col="blue")
 points(arm_data$target_arms_actual_y, arm_data$target_arms_actual_z, xlim=c(-80,80), ylim=c(0,100), col="green")
 points(both_data$target_head_actual_y,both_data$target_head_actual_z, xlim=c(-80,80), ylim=c(0,100), col="orange", pch=1)
 points(both_data$target_head_actual_y,both_data$target_head_actual_z, xlim=c(-80,80), ylim=c(0,100), col="orange", pch=4)
dev.off()

pdf("~/Desktop/figs/head_y_des_actual.pdf")
plot(head_data$target_desired_y, head_data$target_head_actual_y, col="blue", asp=1)
points(arm_data$target_desired_y, arm_data$target_arms_actual_y, col="green")
abline(0,1)
dev.off()

pdf("~/Desktop/figs/head_z_des_actual.pdf")
plot(subset(head_data, target_head_actual_z>0)$target_desired_z, subset(head_data, target_head_actual_z>0)$target_head_actual_z, col="blue", asp=1)
points(subset(arm_data, target_arms_actual_z>0)$target_desired_z, subset(arm_data, target_arms_actual_z>0)$target_arms_actual_z, col="green")
abline(0,1)
dev.off()

for(i in c(48))
{
pdf(paste("~/Desktop/figs/line",i,".pdf", sep=""))
head_data_line = subset(head_data, target_desired_z>(i-5) & target_desired_z<(i+5))
arm_data_line = subset(head_data, target_desired_z>(i-5) & target_desired_z<(i+5))
both_data_line = subset(head_data, target_desired_z>(i-5) & target_desired_z<(i+5))
plot(head_data_line$target_desired_y, abs(head_data_line$target_desired_y-head_data_line$target_perceived_y), col="blue")
points(arm_data_line$target_desired_y, abs(arm_data_line$target_desired_y-arm_data_line$target_perceived_y), col="green")
points(both_data_line$target_desired_y, abs(both_data_line$target_desired_y-both_data_line$target_perceived_y), col="orange")
dev.off()
#hist finish this up TODO
pdf(paste("~/Desktop/figs/hist",i,".pdf", sep=""))
plot(head_data_line$target_desired_y, abs(head_data_line$target_desired_y-head_data_line$target_perceived_y), col="blue")
points(arm_data_line$target_desired_y, abs(arm_data_line$target_desired_y-arm_data_line$target_perceived_y), col="green")
points(both_data_line$target_desired_y, abs(both_data_line$target_desired_y-both_data_line$target_perceived_y), col="orange")
dev.off()
}
