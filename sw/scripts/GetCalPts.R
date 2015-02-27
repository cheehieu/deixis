MeanMatrixCols <- function(mat) {
#means of cols in a matrix
mat_mean = c()
for(i in c(1:ncol(mat))) mat_mean = c(mat_mean,mean(mat[,i], na.rm=TRUE))
return(mat_mean)
}

distance2 <- function (x1, x2, y1, y2) {
   tempX <- x1 - x2
   tempY <- y1 - y2
   return(sqrt(tempX*tempX + tempY*tempY))
}

GetCalPts <- function(part) {
	#head <- subset(part, modality=="1")
	#arm <- subset(part, modality=="4")
	arm_cross <- subset(part, modality=="4")
	
	#cal <- subset(head, target_desired_y == 9 | 						 target_desired_y == -27) 
	#cal <- subset(arm, target_desired_y == 9 | 						   target_desired_y == 27) 
	cal <- subset(arm_cross, target_desired_y == -9 | 						   target_desired_y == -27) 
	return(cal)
}


CalcError <- function(data, part_num) {
	near = subset(data, participant_id==part_num & target_desired_z==51)	
	far = subset(data, participant_id==part_num & target_desired_z==33)

	return(distance2(near$target_desired_y, near$target_perceived_y, near$target_desired_z, near$target_perceived_z))
}



#calPts <- GetCalPts(field)
#numPart = length(unique(calPts$participant_id))-c(1)
#lAccum = c()
#for(i in c(0:numPart))
#{		
#	temp <- CalcError(calPts, i)
#	lAccum = c(lAccum, length(temp))
#}

#accum <- matrix(ncol=numPart+1, nrow=max(lAccum))
#for(i in c(0:numPart))
#{
#	temp <- CalcError(calPts, i)
#	accum[1:length(temp),i+1] <- temp
#}

#accum <- accum[0:8,2:17]
#for(i in c(1:17)) summary(accum[i,])


#to run this source this file
#then accum will contain a matrix whose columns are participant
#error as calculated in CalcError.
#then run 
#my_means = MeanMatrixCols(accum)
#to get a set of means for each column then
#hist(hcnpe_means, xlab="Mean error in inches", ylab="Number of participants", main="Perceived mean error for head modality & near calibration point")
