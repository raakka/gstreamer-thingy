GST_INC    := -I/usr/include/opencv4/ -I/usr/include/gstreamer-1.0 -I/usr/include/glib-2.0 -I/usr/lib/aarch64-linux-gnu/glib-2.0/include -I/usr/lib/aarch64-linux-gnu/gstreamer-1.0/include 
LOCAL_INC  := -Isrc/opencv-code/ -Isrc/utility/ -I/usr/local/ntcore/src/main/native/include/networktables/  -I/usr/local/ntcore/src/main/native/include/networktables/ -I/usr/local/ntcore/src/main/native/include
NTCORE_INC := -I/3419/ntcore/include/ -I/3419/ntcore/wpiutil/include/
INC        := ${NTCORE_INC} ${LOCAL_INC} ${GST_INC}

# LIBS       := -L/3419/ntcore/ `pkg-config --cflags --libs opencv gstreamer-1.0` -lgstapp-1.0 -lgstriff-1.0 -lgstbase-1.0 -lgstvideo-1.0 -lgstpbutils-1.0 -lntcore -lwpiutil -lpthread

# LIBS       := -lopencv_core -lgstapp-1.0 -lgstriff-1.0 -lgstbase-1.0 -lgstvideo-1.0 -lgstpbutils-1.0 -lpthread -lstdc++ -lopencv_imgproc -lopencv_highgui -lopencv_videoio -lopencv_features2d -lopencv_objdetect -lopencv_ml -lopencv_video -lopencv_calib3d -lopencv_objdetect -lopencv_stitching -lopencv_imgcodecs -lopencv_features2d 


LIBS       := -lgstapp-1.0 -lgstriff-1.0 -lgstbase-1.0 -lgstvideo-1.0 -lgstpbutils-1.0 -lpthread -lstdc++ -lopencv_imgproc -lopencv_highgui -lopencv_videoio -lopencv_features2d -lopencv_objdetect -lopencv_ml -lopencv_video -lopencv_calib3d -lopencv_objdetect -lopencv_stitching -lopencv_imgcodecs -lopencv_features2d -lgstreamer-1.0 -lgobject-2.0 -lglib-2.0 -lopencv_core


SOURCES    := $(shell find src -type f -name *.cpp)
OBJECTS    := $(patsubst src/%,obj/%,$(SOURCES:.cpp=.o))

# CPP_ARGS  := -std=c++11 -Wall -Werror -g
CPP_ARGS  := -std=c++11 -g

all: obj gstream_cv

obj:
	@mkdir -p obj/

run: gstream_cv
	./gstream_cv

obj/%.o: src/%.cpp
	@mkdir -p $(dir $@)
	g++ $(CPP_ARGS) $(INC) -c -o $@ $<
	@g++ $(CPP_ARGS) ${INC} -MM src/$*.cpp > obj/$*.d
	@cp -f obj/$*.d obj/$*.d.tmp
	@sed -e 's|.*:|obj/$*.o:|' < obj/$*.d.tmp > obj/$*.d
	@sed -e 's/.*://' -e 's/\\$$//' < obj/$*.d.tmp | fmt -1 | sed -e 's/^ *//' -e 's/$$/:/' >> obj/$*.d
	@rm -f obj/$*.d.tmp

#need to use libtool because of gstreamer libraries
gstream_cv: ${OBJECTS}
	# libtool --mode=link g++ $(CPP_ARGS) ${INC} ${LIBS} -o gstream_cv ${OBJECTS}
	g++ $(CPP_ARGS) ${INC} -o gstream_cv ${OBJECTS} ${LIBS}


clean:
	-rm gstream_cv
	-rm -rf obj

.PHONY: clean run




