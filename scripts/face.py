#!/usr/bin/env python

import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk
from gi.repository import Gdk
from gi.repository import GObject

from threading import Lock

import cairo
import math
import random as rand
import numpy as np
import time

import rospy

from std_msgs.msg import Bool
from pishu_msgs.msg import EmotionalState, MultiFaces, FacePosition

pi = math.pi

context = {}
context_lock  = Lock()

class Eye:

    def __init__(self):

        self.pos_pupil = [0, 0]
        self.desired_pos_pupil = [0, 0]
        self.pupil_saccade = [0, 0]
        self.pupil_radius = 0.1
        self.radius = 0
        self.angle = 0
        self.a = 1 #distortion

        # eye lids: all define the vertical pos, the first one is absolute coord.
        # the other ones are relatie to the first point
        self.upper_eyeside = np.array([0.12, 0.25, 0.25, 0.0])
        self.lower_eyeside = np.array([-0.20, -0.1, -0.1, 0.0])

        # Matrix that contains the values of the different parameters as function
        # of the emotions states. Each column corresponds to an emotional 
        # state, each row corresponds to a parameter
        # columns: neutral, happy, angry, interest
        # rows: upper_eyeside (4), lower_eyeside (4), pupil size (1)
        self.emo_mat = np.zeros((9, 4))

        # params for neutral
        self.emo_mat[:, 0] = np.array([0.35, -0.25, -0.25, 0.0,
                                                         -0.35, 0.2, 0.2, 0.0,
                                                         0.08])

        # params for angry
        self.emo_mat[:, 2] = np.array([0.12, 0.25, 0.25, 0.0,
                                                         -0.20, -0.1, -0.1, 0.0,
                                                         0.06])

        print self.emo_mat

        self.prev_time = time.time()

        self.pos_update_counter = 0
        self.saccade_update_counter = 0

    def update(self):

        new_time = time.time()
        ellapsed = new_time - self.prev_time
        self.prev_time = new_time

        # update emotion

        context['emo_vect'] = 0.8*context['emo_vect'] + 0.2*context['new_emo_vect'] 

        # update eyesides

        print context['emo_vect']

        new_eyesides = np.dot(self.emo_mat, context['emo_vect'])
    
        self.upper_eyeside = new_eyesides[0:4]
        self.lower_eyeside = new_eyesides[4:8]

        print self.lower_eyeside

        if self.pos_update_counter < 0:
            radius = 0.22*rand.random() 
            angle = 2*pi*rand.random() 

            self.radius = radius*math.cos(angle) + 0.3*radius*math.sin(angle)
            self.angle = angle

            self.a = 1.0 - 6.25*radius*radius # so that 6.25*0.4^2 = 1 (eye size = 0.4)

            self.desired_pos_pupil = [radius*math.cos(angle), radius*math.sin(angle)]

            self.pos_update_counter = 0.04*np.random.poisson(20, 1)

            #print self.desired_pos_pupil 

#		# Saccades this way does not work, makes the eye too jittery
#		# Maybe use for some states (e.g. angry)
#		if self.saccade_update_counter < 0:
#
#			self.pupil_saccade = [0.001*rand.gauss(0, 1), 0.001*rand.gauss(0, 1)]
#
#			self.saccade_update_counter = 0.04*np.random.poisson(4, 1)

        alpha = math.exp(-0.5*ellapsed/0.05)
        #print ellapsed

        self.pos_pupil[0] = (1-alpha)*self.desired_pos_pupil[0] + alpha*self.pos_pupil[0]
        self.pos_pupil[1] = (1-alpha)*self.desired_pos_pupil[1] + alpha*self.pos_pupil[1]

        self.pos_update_counter -= ellapsed
#		self.saccade_update_counter -= ellapsed


	

        

    def pupil_mat(self):

        mat = cairo.Matrix(self.a*math.cos(self.angle), self.a*math.sin(self.angle),
            -math.sin(self.angle), math.cos(self.angle),
            0.5 + self.pos_pupil[0],
            0.5 + self.pos_pupil[1])

        return mat

    def iris_mat(self):

        mat = cairo.Matrix(self.a*math.cos(self.angle), self.a*math.sin(self.angle),
            -math.sin(self.angle), math.cos(self.angle),
            0.5 + 0.8*self.pos_pupil[0],
            0.5 + 0.8*self.pos_pupil[1])

        return mat

class EyeWindow(Gtk.Window):

    def __init__(self):
        super(EyeWindow, self).__init__()

        #init ROS

        rospy.init_node('face', anonymous=False)

        rospy.Subscriber("cortex/emotion", EmotionalState, emotion_callback)

        self.pub_face_touched = rospy.Publisher("face/touched", Bool, queue_size = 5)

        self.eye = Eye()

        self.init_ui()

        self.event_last_time = 0 # needed to not detect events twice


    def init_ui(self):    

        darea = Gtk.DrawingArea()
        darea.connect("draw", self.on_draw)
        self.add(darea)

        self.set_title("The Eye")
        #self.resize(600, 600)
        self.fullscreen()
        self.set_position(Gtk.WindowPosition.CENTER)
        self.connect("delete-event", Gtk.main_quit)
        self.connect("realize", self.realize_cb)
        self.connect("button-press-event", self.on_mouse_click)
        self.set_events(self.get_events() |
                                  Gdk.EventMask.BUTTON_PRESS_MASK)

        GObject.timeout_add(100, self.timeout)
        self.show_all()

    def realize_cb(self, w):
        cursor = Gdk.Cursor.new(Gdk.CursorType.BLANK_CURSOR)
        self.get_window().set_cursor(cursor)

    def on_mouse_click(self, w, event):
        if self.event_last_time != event.time:
            self.event_last_time = event.time
            self.pub_face_touched.publish(True)


    def timeout(self):
        if rospy.is_shutdown():
            exit()

        self.queue_draw()
        return True

    def on_draw(self, wid, cr):
	
        # Draw background

        cr.save()
        cr.scale(self.get_size()[0], self.get_size()[0])

        cr.set_source_rgb(0, 0, 0)
        cr.rectangle(0, 0, 1, 1)
        cr.fill()
        cr.restore()

        # Set the coordinate system to only draw in the center
        # Scale so that the drawing space goes from 0 to 1

        cr.translate((self.get_size()[0] - self.get_size()[1])/2, 0)

        cr.scale(self.get_size()[1], self.get_size()[1])

        # Draw the sclera (the white of the eye)
        radial = cairo.RadialGradient(0.47, 0.47, 0.32, 0.50, 0.50, 0.46)
        radial.add_color_stop_rgb(0,  0.98, 0.95, 0.98)
        radial.add_color_stop_rgb(1,  0.57, 0.55, 0.57)
        cr.set_source(radial)
        cr.arc(0.5, 0.5, 0.4, 0, 2*pi)
        cr.fill()

        self.eye.update()

        mat_pupil = self.eye.pupil_mat()

        mat_iris = self.eye.iris_mat()

        # Draw the iris	

        cr.save()
        cr.transform(mat_iris)
        radial = cairo.RadialGradient(0, 0, 0.07,
			         0, 0, 0.3)
        radial.add_color_stop_rgb(0,  0, 0.6, 1.0)
        radial.add_color_stop_rgb(1,  0.5, 0.9, 0.8)
        cr.set_source(radial)
        cr.arc(0, 0, 0.28, 0, 2*pi)
        cr.fill()
        cr.restore()

        # Draw the pupil	

        cr.save()
        cr.transform(mat_pupil)
        cr.set_source_rgb(0.05, 0.02, 0)
        cr.arc(0, 0, self.eye.pupil_radius, 0, 2*pi)
        cr.fill()
        cr.restore()

        # Draw the lower eyeside

        print self.eye.lower_eyeside

        cr.set_source_rgb(0, 0, 0)
        cr.move_to(0.1, 1)
        cr.rel_line_to(0, self.eye.lower_eyeside[0])
        cr.rel_curve_to(0.3, self.eye.lower_eyeside[1], 
                                (0.8-0.3), self.eye.lower_eyeside[2], 
                                0.8, self.eye.lower_eyeside[3])
        #cr.rel_curve_to(0.3, -0.1, (0.8-0.3), -0.1, 0.8, 0.0) #angry look
        cr.rel_line_to(0, 0.35)
        cr.close_path()
        cr.fill()

        cr.set_source_rgb(0.7, 0.2, 0)
        cr.move_to(0.1, 1 + self.eye.lower_eyeside[0]+0.01)
        cr.rel_curve_to(0.3, self.eye.lower_eyeside[1], 
                                (0.8-0.3), self.eye.lower_eyeside[2], 
                                0.8, self.eye.lower_eyeside[3])
        cr.rel_line_to(0, -0.05)
        cr.rel_curve_to(-0.3, self.eye.lower_eyeside[2]-self.eye.lower_eyeside[3],
                                -(0.8-0.3), self.eye.lower_eyeside[1]-self.eye.lower_eyeside[3], 
                               -0.8, -self.eye.lower_eyeside[3])
        cr.close_path()
        cr.fill()

        # Draw the upper eyeside

        cr.set_source_rgb(0, 0, 0)
        cr.move_to(0.1, 0)
        cr.rel_line_to(0, self.eye.upper_eyeside[0])
        cr.rel_curve_to(0.3, self.eye.upper_eyeside[1], 
                                (0.8-0.3), self.eye.upper_eyeside[2], 
                                0.8, self.eye.upper_eyeside[3])
        #cr.rel_curve_to(0.3, 0.25, (0.8-0.3), 0.25, 0.8, 0.0) #angry look
        #cr.rel_curve_to(0.3, 0.25, (0.8-0.3), -0.25, 0.8, 0.1) #surprized look
        cr.rel_line_to(0, -0.35)
        cr.close_path()
        cr.fill()

        cr.set_source_rgb(0.7, 0.2, 0)
        cr.move_to(0.1, self.eye.upper_eyeside[0]+0.01)
        cr.rel_curve_to(0.3, self.eye.upper_eyeside[1], 
                                (0.8-0.3), self.eye.upper_eyeside[2], 
                                0.8, self.eye.upper_eyeside[3])
        cr.rel_line_to(0, -0.05)
        cr.rel_curve_to(-0.3, self.eye.upper_eyeside[2]-self.eye.upper_eyeside[3],
                                -(0.8-0.3), self.eye.upper_eyeside[1]-self.eye.upper_eyeside[3], 
                               -0.8, -self.eye.upper_eyeside[3])
        cr.close_path()
        cr.fill()


def emotion_callback(data):
    context_lock.acquire()
    context['happy'] = data.happy
    context['angry'] = data.angry
    context['interest'] = data.interest
    new_emo = np.array([0, data.happy, data.angry, data.interest])
    new_emo[0] = max(0, 1 - np.sum(new_emo))
    context['new_emo_vect'] = new_emo
    context_lock.release()

def main():
    context_lock.acquire()
    context['happy'] = 0
    context['angry'] = 0
    context['interest'] = 0
    context['new_emo_vect']  = np.array([0, 0, 1, 0])
    context['emo_vect']  = np.array([1, 0, 0, 0])
    context_lock.release()

    app = EyeWindow()

    Gtk.main()

           
        
if __name__ == "__main__":    
    main()	





