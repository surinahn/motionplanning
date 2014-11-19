#Makefile for Java programming
# 	Do not try to combine Java with C/C++.  Too different!

###################################################
# VARIABLES
###################################################
# 	THIS SECTION CONTAINS THE ONLY CUSTOMIZATION YOU NEED
# 	BUT see the -include command at end of this section!
#	It may override all the defaults in this Section
###################################################
# main file
p=PrintFile
p=ReadFile
p=QuadTester2
p=QuadTester

# TOP LEVEL CHOICE TASK: edit or compile
d=vi
d=applet
d=javac
d=q

# special flags
l-flags= -lcurses

# default arguments for testing program
arg=10
arg2=1

# arguments for Java program:
	file = bugtrap.txt	# bugtrap -- a bit slow for a default input file
	file = input2.txt	# simple input, better for default input file
	dir = ../inputData/	# WARNING: dirField must end with "/"
	strat = 0
	rad = 40
	eps = 4
	startx = 50
	starty = 50
	goalx = 400
	goaly = 500


# INCLUDE a (permanent) local file.
#   -- The above defaults may be overridden by the next inclusion!
#   -- Note: "-include foo" is used instead of "include foo"
#           so that if the file "foo" is missing, no error is generated.
#
# Note: p should be set in make.preclude...

-include make.preclude

######################################################
# YOU SHOULD NOT HAVE TO CUSTOMIZE BELOW THIS LINE!
######################################################

# TOP LEVEL Targets
ifeq ($(d),vi) # 1
	ext=.java
else 
ifeq ($(d),java) # 2
	ext=.java
else
ifeq ($(d),applet) # 3
	ext=.html
else
	ext=
endif #3
endif #2
endif #1

ext=.java

pclass := $(p).class
###################################################
# TARGETS
###################################################
default: $(d)

# compile
c compile javac:
	javac ${p}.java

# run or execute QUADTESTER
rq runq: 
	java ${p} \
	        ${file} $(dir) \
		$(strat) \
		$(rad) $(eps) \
		$(startx) $(starty) \
		$(goalx) $(goaly) \
		&

# run or execute RRT
rr runr:
	java RRT \
	        ${file} $(dir) \
		$(rad) $(eps) \
		$(startx) $(starty) \
		$(goalx) $(goaly) \
		&

# compile and run
# 	THIS IS AN IMPROVEMENT OF the QuadTester target!
q: AStar.class Quadtree.class QuadTester.class\
                Robot.class UnionFind.class Record.class 
	make runq

rrt RRT: AStar.class RRT.class
	make runr

# compile and run 
# 	THIS VERSION forces you to ALWAYS compile the 6 files!!
# 	See the target "rq" for a better version.
#
quad : Quadtree.java QuadTester.java \
                Robot.java UnionFind.java Record.java AStar.java
	javac Robot.java
	javac Record.java
	javac UnionFind.java
	javac AStar.java
	javac Quadtree.java
	javac QuadTester.java
	make run


##################################################
# Standard examples
##################################################
eg: QuadTester.class
	@echo "Eg: Default example.  You may override any or all default arguments:"
	@echo "     file, dir, strat, rad, eps, startx, starty, goalx, \ 				goaly."
	java QuadTester \
	        ${file} $(dir) \
		$(strat) \
		$(rad) $(eps) \
		$(startx) $(starty) \
		$(goalx) $(goaly) \
		&

eg0: QuadTester.class
	@echo "Eg0: Simple instance with path with eps=4"
	java QuadTester \
		input2.txt ../inputData/ \
		0 \
		40 4 \
		50 50 \
		400 500 \
		&

eg0a: QuadTester.class
	@echo "Eg0a: This bugtrap instance with NO path with eps=20"
	java QuadTester \
		input2.txt ../inputData/ \
		0 \
		40 20 \
		50 50 \
		400 530 \
		&

bug eg1: QuadTester.class
	@echo "Eg1: This bugtrap instance has a path with eps=4"
	java QuadTester \
		bugtrap.txt ../inputData/ \
		0 \
		10 4 \
		200 350 \
		30 30 \
		&

buga eg1a: QuadTester.class
	@echo "Eg1a: This bugtrap instance has NO PATH with eps=5"
	java QuadTester \
		bugtrap.txt ../inputData/ \
		0 \
		10 5 \
		200 350 \
		30 30 \
		&


##################################################
# Applet
##################################################
a applet: ${p}.html
	appletviewer ${p}.html
b:
	@echo \" this \"

# WITH THIS, YOU CAN VIEW ANY APPLET INSTANTLY, AS THE HTML FILE
# IS AUTOMATICALLY GENERATED!
ia instantapplet: 
	@echo $(pclass)
	@echo "<html>" > instantApplet.html
ifeq ($(myOS),unix)
	@echo "<!-- 	THIS FILE IS AUTO GENERATED -->" >> instantApplet.html
	@echo "<title> Instant Applet </title>" >> instantApplet.html
	@echo "<body bgcolor=lightgreen>" >> instantApplet.html
	@echo "<center> <h2> Instant Applet </h2></center>" >> instantApplet.html
	@echo "<applet code=" >> instantApplet.html
	@echo ${pclass} >> instantApplet.html
	@echo "WIDTH=500 HEIGHT=300></applet>" >> instantApplet.html
	@echo "</body></html>" >> instantApplet.html
else
	@echo "<!-- 	THIS FILE IS AUTO GENERATED -->" >> instantApplet.html
	@echo "<title> Instant Applet </title>" >> instantApplet.html
	@echo "<body bgcolor=lightgreen>" >> instantApplet.html
	@echo "<center> <h2> Instant Applet </h2></center>" >> instantApplet.html
	@echo "<applet code=" >> instantApplet.html
	@echo ${pclass} >> instantApplet.html
	@echo " WIDTH=500 HEIGHT=300>" >> instantApplet.html
	@echo "</applet>" >> instantApplet.html
	@echo "</body></html>" >> instantApplet.html
endif
	appletviewer instantApplet.html

t test:	
ifeq ($(pf),cyg)
	a.exe ${args}
else
	a.out ${args}
endif

x:
	pclass=$(p).class
	@echo $(pclass)

# sample:
$(phony convexHull):
	gcc ${flags} convexHull.c -o convexHull${exe}
	convexHull ${args}

###################################################
# editing/printing
###################################################
vi:
	gvim ${p}${ext} &

a1:
	a2ps -Pnhp23 ${p}${ext} &

a8:
	a2ps -Pnhp8 ${p}${ext} &

#=================================================
# Rules
#=================================================
# the first ".SUFFIXES" clears out all suffixes, the second establishes new ones!
.SUFFIXES:
.SUFFIXES: .java .class

%: %.o
	${CXX} $(LDFLAGS) $< $(CORE_LIB) -o $@

.cpp.o:
	${CXX} -c $(CXXFLAGS) $(CORE_INC) $< -o $@

.java.class:
	javac $< 

#=================================================
# Clean object files
#=================================================
clean:
	-@test -z "*.class" || rm -f *.class
	-@test -z "*~" || rm -f *~

#=================================================
# Remove executable files
#=================================================
EXEPROGS=$(TARGETS:=$(EXETYPE))
MORE_EXEPROGS=$(MORE_TARGETS:=$(EXETYPE))

vclean veryclean: clean
	-@test -z "$(EXEPROGS)" || rm -f $(EXEPROGS)
	-@test -z "$(MORE_EXEPROGS)" || rm -f $(MORE_EXEPROGS)
	-@test -z iA.html || rm -f iA.html

###################################################
# housekeeping
###################################################
m:
	gvim Makefile &

tar:
	tar -cvf fork.tar \
		Makefile *.c shellScript

######################################################
# Query
######################################################
show options param par:
	-@echo "VARS: =============================="
	-@echo "d=" $(d)
	-@echo "FILES: ============================="
	-@echo "p=" $(p) ", pp=" $(pp)
	-@echo "q=" $(q) ", qq=" $(qq)
	-@echo "SEARCH: =============="
	-@echo "ss=" $(ss)
	-@echo "aa=" $(aa) ", bb=" $(bb) ", ig=" $(ig)
	-@echo "sfiles=" $(sfiles) ", sdir=" $(sdir)
	-@echo "FTP/CVS: =============="
	-@echo "f=" $(f) ", curpath=" $(curpath) ", figfile=" $(figfile)
	-@echo "tarfile=" $(tarfile) ", msg=" ${msg}
	-@echo "PRINTING OPTIONS==================="
	-@echo "prfile=" $(prfile) ", prtype=" $(prtype)
	-@echo "priparam par:
	-@echo "OTHERS ==================="
	@echo "PATH = " $$PATH

###################################################
# Postclusion of files
###################################################
-include make.postclude

###################################################
# END
###################################################
