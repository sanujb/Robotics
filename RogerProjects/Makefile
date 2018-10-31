# replace this with the top of your X11 tree
# X11 = /exp/rcf/share/X11R5
X11 = /usr/X11R6

############## do not change below this line ####################

ROGERINCDIR = ./include

XINCDIR = $(X11)/include
XLIBDIR = $(X11)/lib 
EDLAB_XINCDIR = /usr/include/X11
EDLAB_XLIBDIR = /usr/lib/i386-linux-gnu

XAWLIB = -lXaw
XMULIB = -lXmu
XTOOLLIB = -lXt
XLIB = -lX11
XEXTLIB = -lXext
MATHLIB = -lm

LIBS =  -L$(XLIBDIR) $(XAWLIB) $(XMULIB) $(XTOOLLIB) $(XLIB) $(XEXTLIB) \
	$(MATHLIB)

RM = rm -f
CC = gcc
CCFLAGS = -c -g -I. -I$(XINCDIR) -I$(EDLAB_XINCDIR) -I$(ROGERINCDIR)

.SUFFIXES:	.c	.o

.c.o:	
	$(CC) $(CCFLAGS) $<

############## do not change above this line ####################

PROG = x

PROJECTOFILES = project1-MotorUnits/project1.o \
                project2-ArmKinematics/project2.o \
		project3-Vision/project3.o \
		project4-SearchTrack/project4.o \
                project4-SearchTrack/sampling.o \
		project5-StereoKinematics/project5.o \
		project6-Kalman/project6.o \
		project6-Kalman/kalman.o \
                project7-ChasePunch/project7.o \
		project8-PathPlanning/project8.o \
		project9-Grasp/project9.o \
                project10-Model/project10.o \
		project11-Belief/project11.o

HFILES = roger.h simulate.h control.h modes.h matrix_math.h

all:  subdirs  $(PROG)

subdirs:
	cd project1-MotorUnits; make; cd ..; \
	cd project2-ArmKinematics; make; cd ..;
	cd project3-Vision; make; cd ..; \
	cd project4-SearchTrack; make; cd ..; \
	cd project5-StereoKinematics; make; cd ..;
	cd project6-Kalman; make; cd ..; \
	cd project7-ChasePunch; make; cd ..;
	cd project8-PathPlanning; make; cd ..; \
	cd project9-Grasp; make; cd ..;
	cd project10-Model; make; cd ..;
	cd project11-Belief; make; cd ..;

$(PROG): $(PROJECTOFILES) 
	$(CC) -o $@ lib/simulator.a $(LIBS) $^

clean:
	$(RM) $(PROJECTOFILES) $(PROG) *~


