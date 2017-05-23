#!/usr/bin/env python




import pygame
from pygame.locals import *

import rospy



pygame.init()

display_width = 1920
display_height = 1080

white = (255,255,255)
black = (0,0,0)
green = (0,255,0)



gameDisplay = pygame.display.set_mode((display_width,display_height), RESIZABLE, 32)
pygame.display.set_caption('On Screen Display')
clock = pygame.time.Clock()

rospy.init_node('ros_time', anonymous=True)



font_3 = pygame.font.Font('/home/lourenco/Downloads/gill_sans_MT.ttf',220)

old_dig = 0

def movie():
	
	print rospy.Time.now()
	gameDisplay.fill(black)
	
	time = str(rospy.Time.now())
	
	start = rospy.Time.now().to_nsec()
	#print type(start)
	while rospy.Time.now().to_nsec()-start < 30000:
		#print "fdsfs"
		pass
				
		#print rospy.Time.now().to_nsec()-start
		
	s = time[6:12]
	#print s
	new_dig = int(s[3])
	
	#print new_dig
	#print old_dig
	#a = new_dig - old_dig
	#print c
	#if new_dig != old_dig:
	#	c = c + 1
	#if c==3:
	#	print "ola"
	#	c=0
		
	
	#print a
	#if new_dig - old_dig <4:
	#	new_dig = old_dig
	#	print new_dig
	#elif new_dig - old_dig >2:
	#	pass
	
	
	#a = str(s[0])+" "+str(s[1])+" "+str(s[2])+" "+str(s[3])+" "+str(s[4])+" "+str(s[5])
	a = str(s[2])+" "+str(s[3])+" "+str(s[4])+" "+str(s[5]) 
	hora_teste = font_3.render('%s'%a, True, white)
	

		
	rotate = gameDisplay
	rotate.blit(hora_teste,(((display_width)/2)-380,(display_height/2)-150))
	
	pygame.display.update()

	pygame.display.set_caption("FPS: "+ str(clock.get_fps()))
	clock.tick(0.5)
	return new_dig
	for event in pygame.event.get():
		if event.type == QUIT:
		    pygame.quit()
		    sys.exit()
		if event.type == KEYDOWN:
			if event.key == K_ESCAPE:
				pygame.quit()
				sys.exit()

	
while True:
	#old_dig = movie()
	#start = rospy.Time.now().to_nsec()
	#print type(start)
	#while rospy.Time.now().to_nsec()-start < 30000:
		#print "fdsfs"
	#	pass
	#print old_dig
	movie()
	
