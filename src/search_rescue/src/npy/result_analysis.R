library(reticulate)
library(ggplot2)
library(dplyr)
np <- import("numpy")
library("qpcR")  
# data reading


gg_col <- function(n) {
  hues = seq(15, 375, length = n + 1)
  hcl(h = hues, l = 65, c = 100)[1:n]
}

setwd('/home/samprince/search_rescue/src/search_rescue/src/npy')

mat31 <- np$load("J3-1.npy")
mat32 <- np$load("J3-2.npy")
mat33 <- np$load("J3-3.npy")
mat34 <- np$load("J3-4.npy")
mat35 <- np$load("J3-5.npy")
mat36 <- np$load("J3-6.npy")
mat37 <- np$load("J3-7.npy")
mat38 <- np$load("J3-8.npy")
mat39 <- np$load("J3-9.npy")
mat310 <- np$load("J3-10.npy")

mat21 <- np$load("J2-1.npy")
mat22 <- np$load("J2-2.npy")
mat23 <- np$load("J2-3.npy")
mat24 <- np$load("J2-4.npy")
mat25 <- np$load("J2-5.npy")
mat26 <- np$load("J2-6.npy")
mat27 <- np$load("J2-7.npy")
mat28 <- np$load("J2-8.npy")
mat29 <- np$load("J2-9.npy")
mat210 <- np$load("J2-10.npy")


my_data31 <- as.data.frame.table(mat31,keep.rownames=TRUE)
my_data32 <- as.data.frame.table(mat32,keep.rownames=TRUE)
my_data33 <- as.data.frame.table(mat33,keep.rownames=TRUE)
my_data34 <- as.data.frame.table(mat34,keep.rownames=TRUE)
my_data35 <- as.data.frame.table(mat35,keep.rownames=TRUE)
my_data36 <- as.data.frame.table(mat36,keep.rownames=TRUE)
my_data37 <- as.data.frame.table(mat37,keep.rownames=TRUE)
my_data38 <- as.data.frame.table(mat38,keep.rownames=TRUE)
my_data39 <- as.data.frame.table(mat39,keep.rownames=TRUE)
my_data310 <- as.data.frame.table(mat310,keep.rownames=TRUE)

my_data21 <- as.data.frame.table(mat21,keep.rownames=TRUE)
my_data22 <- as.data.frame.table(mat22,keep.rownames=TRUE)
my_data23 <- as.data.frame.table(mat23,keep.rownames=TRUE)
my_data24 <- as.data.frame.table(mat24,keep.rownames=TRUE)
my_data25 <- as.data.frame.table(mat25,keep.rownames=TRUE)
my_data26 <- as.data.frame.table(mat26,keep.rownames=TRUE)
my_data27 <- as.data.frame.table(mat27,keep.rownames=TRUE)
my_data28 <- as.data.frame.table(mat28,keep.rownames=TRUE)
my_data29 <- as.data.frame.table(mat29,keep.rownames=TRUE)
my_data210 <- as.data.frame.table(mat210,keep.rownames=TRUE)

data_base <- qpcR:::cbind.na(my_data31$Freq, my_data32$Freq, my_data33$Freq, my_data34$Freq,my_data35$Freq, my_data36$Freq, my_data37$Freq, my_data38$Freq,my_data39$Freq, my_data310$Freq,
                             my_data21$Freq, my_data22$Freq, my_data23$Freq, my_data24$Freq,my_data25$Freq, my_data26$Freq, my_data27$Freq, my_data28$Freq,my_data29$Freq, my_data210$Freq)
colnames(data_base) <- c('r31','r32','r33','r34','r35','r36','r37','r38','r39','r310',
                         'r21','r22','r23','r24','r25','r26','r27','r28','r29','r210')
data_base<- data.frame(data_base)
data_base[32,12] = NA
Data_r3 <- data_base %>% dplyr::select(1:10)
r3_means <- rowMeans(Data_r3,na.rm = TRUE)
data_base$r3_means <- r3_means   

Data_r2 <- data_base %>% dplyr::select(11:20)
r2_means <- rowMeans(Data_r2,na.rm = TRUE)
data_base$r2_means <- r2_means   




ggplot(data_base, aes(x=1:nrow(data_base))) + 
  geom_line(aes(y = r31), color = "darkred") + 
  geom_line(aes(y = r32), color = "darkred") + 
  geom_line(aes(y = r33), color = "darkred") + 
  geom_line(aes(y = r34), color = "darkred") + 
  geom_line(aes(y = r21), color="steelblue", linetype="twodash") +
  geom_line(aes(y = r22), color="steelblue", linetype="twodash") +
  geom_line(aes(y = r23), color="steelblue", linetype="twodash") +
  geom_line(aes(y = r24), color="steelblue", linetype="twodash") 

colors <- c("three robots" = gg_col(1), "two robots" = gg_col(2))
setEPS()
postscript("cost_series.eps")
ggplot(data_base, aes(x=1:nrow(data_base))) + 
  geom_line(aes(y = r3_means, color = "three robots")) + 
  geom_line(aes(y = r2_means, color="two robots")) +
  labs(x = "Iteration #", y = "Average Cost",color = "Legend") +
  theme(legend.position=c(0.85,0.9),aspect.ratio=1,text = element_text(size =14))

dev.off()