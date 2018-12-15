clear all
load('hw2_sample_map.mat')
bug = Bug1(simplemap0);
bug.query([60,20],[60,100],'animate','current')
