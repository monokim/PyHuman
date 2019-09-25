import pygame
import pymunk
import pymunk.pygame_util
import os
import math
import sys
import neat
import random

screen_width = 1900
screen_height = 960
space = pymunk.Space()
space.gravity = (0.0, -900.0)
generation = 0

class Robot:
    def __init__(self):
        self.tick = 0

        moment = 10
        friction = 0.5

        self.shape = pymunk.Poly.create_box(None, (50, 100))
        body_moment = pymunk.moment_for_poly(moment, self.shape.get_vertices())
        self.body = pymunk.Body(moment, body_moment)
        #self.body = pymunk.Body(body_type=pymunk.Body.STATIC)

        self.body.position = (200, 350)
        #self.body.position = (screen_width/2, screen_height - 500)
        self.shape.body = self.body
        self.shape.color = (150, 150, 150, 0)

        head_moment = pymunk.moment_for_circle(moment, 0, 30)
        self.head_body = pymunk.Body(moment, head_moment)
        self.head_body.position = (self.body.position.x, self.body.position.y+80)
        self.head_shape = pymunk.Circle(self.head_body, 30)
        self.head_shape.friction = friction
        self.head_joint = pymunk.PivotJoint(self.head_body, self.body, (-5, -30), (-5, 50))
        self.head_joint2 = pymunk.PivotJoint(self.head_body, self.body, (5, -30), (5, 50))


        arm_size = (100, 20)
        self.left_arm_upper_shape = pymunk.Poly.create_box(None, arm_size)
        left_arm_upper_moment = pymunk.moment_for_poly(moment, self.left_arm_upper_shape.get_vertices())
        self.left_arm_upper_body = pymunk.Body(moment, left_arm_upper_moment)
        self.left_arm_upper_body.position = (self.body.position.x-30, self.body.position.y)
        self.left_arm_upper_shape.body = self.left_arm_upper_body
        self.left_arm_upper_joint = pymunk.PivotJoint(self.left_arm_upper_body, self.body, (arm_size[0] / 2, 0), (-25, 30))
        self.la_motor = pymunk.SimpleMotor(self.body, self.left_arm_upper_body, 0)

        self.right_arm_upper_shape = pymunk.Poly.create_box(None, arm_size)
        right_arm_upper_moment = pymunk.moment_for_poly(moment, self.right_arm_upper_shape.get_vertices())
        self.right_arm_upper_body = pymunk.Body(moment, right_arm_upper_moment)
        self.right_arm_upper_body.position = (self.body.position.x+30, self.body.position.y)
        self.right_arm_upper_shape.body = self.right_arm_upper_body
        self.right_arm_upper_joint = pymunk.PivotJoint(self.right_arm_upper_body, self.body, (-arm_size[0] / 2, 0), (25, 30))
        self.ra_motor = pymunk.SimpleMotor(self.body, self.right_arm_upper_body, 0)

        thigh_size = (30, 60)
        self.lu_shape = pymunk.Poly.create_box(None, thigh_size)
        lu_moment = pymunk.moment_for_poly(moment, self.lu_shape.get_vertices())
        self.lu_body = pymunk.Body(moment, lu_moment)
        self.lu_body.position = (self.body.position.x-20, self.body.position.y-50)
        self.lu_shape.body = self.lu_body
        self.lu_shape.friction = friction
        self.lu_joint = pymunk.PivotJoint(self.lu_body, self.body, (0, thigh_size[1] / 2), (-20, -50))
        self.lu_motor = pymunk.SimpleMotor(self.body, self.lu_body, 0)

        self.ru_shape = pymunk.Poly.create_box(None, thigh_size)
        ru_moment = pymunk.moment_for_poly(moment, self.ru_shape.get_vertices())
        self.ru_body = pymunk.Body(moment, ru_moment)
        self.ru_body.position = (self.body.position.x+20, self.body.position.y - 50)
        self.ru_shape.body = self.ru_body
        self.ru_shape.friction = friction
        self.ru_joint = pymunk.PivotJoint(self.ru_body, self.body, (0, thigh_size[1] / 2), (20, -50))
        self.ru_motor = pymunk.SimpleMotor(self.body, self.ru_body, 0)

        leg_size = (20, 70)
        self.ld_shape = pymunk.Poly.create_box(None, leg_size)
        ld_moment = pymunk.moment_for_poly(moment, self.ld_shape.get_vertices())
        self.ld_body = pymunk.Body(moment, ld_moment)
        self.ld_body.position = (self.lu_body.position.x, self.lu_body.position.y - 100)
        self.ld_shape.body = self.ld_body
        self.ld_shape.friction = friction
        self.ld_joint = pymunk.PivotJoint(self.ld_body, self.lu_body, (0, leg_size[1] / 2), (0, -thigh_size[1] / 2))
        self.ld_motor = pymunk.SimpleMotor(self.lu_body, self.ld_body, 0)

        self.rd_shape = pymunk.Poly.create_box(None, leg_size)
        rd_moment = pymunk.moment_for_poly(moment, self.rd_shape.get_vertices())
        self.rd_body = pymunk.Body(moment, rd_moment)
        self.rd_body.position = (self.ru_body.position.x, self.ru_body.position.y - 100)
        self.rd_shape.body = self.rd_body
        self.rd_shape.friction = friction
        self.rd_joint = pymunk.PivotJoint(self.rd_body, self.ru_body, (0, leg_size[1] / 2), (0, -thigh_size[1] / 2))
        self.rd_motor = pymunk.SimpleMotor(self.ru_body, self.rd_body, 0)


        foot_size = (45, 20)
        self.lf_shape = pymunk.Poly.create_box(None, foot_size)
        rd_moment = pymunk.moment_for_poly(moment, self.lf_shape.get_vertices())
        self.lf_body = pymunk.Body(moment, rd_moment)
        self.lf_body.position = (self.ld_body.position.x + foot_size[0]/2, self.ld_body.position.y + (foot_size[1]/2 + leg_size[1]/2))
        self.lf_shape.body = self.lf_body
        self.lf_shape.friction = friction
        self.lf_shape.elasticity = 0.1
        self.lf_joint = pymunk.PivotJoint(self.ld_body, self.lf_body, (-5, -leg_size[1] / 2), (-foot_size[0]/2 + 10, foot_size[1]/2))
        self.lf_motor = pymunk.SimpleMotor(self.ld_body, self.lf_body, 0)

        self.rf_shape = pymunk.Poly.create_box(None, foot_size)
        rd_moment = pymunk.moment_for_poly(moment, self.rf_shape.get_vertices())
        self.rf_body = pymunk.Body(moment, rd_moment)
        self.rf_body.position = (self.rd_body.position.x + foot_size[0]/2, self.rd_body.position.y + (foot_size[1]/2 + leg_size[1]/2))
        self.rf_shape.body = self.rf_body
        self.rf_shape.friction = friction
        self.rf_shape.elasticity = 0.1
        self.rf_joint = pymunk.PivotJoint(self.rd_body, self.rf_body, (-5, -leg_size[1] / 2), (-foot_size[0]/2 + 10, foot_size[1]/2))
        self.rf_motor = pymunk.SimpleMotor(self.rd_body, self.rf_body, 0)

        space.add(self.body, self.shape, self.head_body, self.head_shape, self.head_joint, self.head_joint2)
        space.add(self.left_arm_upper_body, self.left_arm_upper_shape, self.left_arm_upper_joint, self.la_motor)
        space.add(self.right_arm_upper_body, self.right_arm_upper_shape, self.right_arm_upper_joint, self.ra_motor)
        space.add(self.lu_body, self.lu_shape, self.lu_joint, self.lu_motor)
        space.add(self.ru_body, self.ru_shape, self.ru_joint, self.ru_motor)
        space.add(self.ld_body, self.ld_shape, self.ld_joint, self.ld_motor)
        space.add(self.rd_body, self.rd_shape, self.rd_joint, self.rd_motor)
        space.add(self.lf_body, self.lf_shape, self.lf_joint, self.lf_motor)
        space.add(self.rf_body, self.rf_shape, self.rf_joint, self.rf_motor)


        shape_filter = pymunk.ShapeFilter(group=1)
        self.shape.filter = shape_filter
        self.head_shape.filter = shape_filter
        self.left_arm_upper_shape.filter = shape_filter
        self.right_arm_upper_shape.filter = shape_filter
        self.lu_shape.filter = shape_filter
        self.ru_shape.filter = shape_filter
        self.ld_shape.filter = shape_filter
        self.rd_shape.filter = shape_filter
        self.lf_shape.filter = shape_filter
        self.rf_shape.filter = shape_filter

        self.face = pygame.image.load('normal.png')
        self.face = pygame.transform.scale(self.face, (100, 100))

        self.red_face = pygame.image.load('red.png')
        self.red_face = pygame.transform.scale(self.red_face, (100, 100))

        self.is_done = False
        self.is_catch = False
        self.distance = 1000

        self.lu_flag = False
        self.ld_flag = False
        self.ru_flag = False
        self.rd_flag = False
        self.la_flag = False
        self.ra_flag = False
        self.lf_flag = False
        self.rf_flag = False

    def get_shapes(self):
        body = self.body, self.shape
        head = self.head_body, self.head_shape, self.head_joint, self.head_joint2
        left_arm = self.left_arm_upper_body, self.left_arm_upper_shape, self.left_arm_upper_joint, self.la_motor
        right_arm = self.right_arm_upper_body, self.right_arm_upper_shape, self.right_arm_upper_joint, self.ra_motor
        left_up_leg = self.lu_body, self.lu_shape, self.lu_joint, self.lu_motor
        left_down_leg = self.ld_body, self.ld_shape, self.ld_joint, self.ld_motor
        left_foot = self.lf_body, self.lf_shape, self.lf_joint, self.lf_motor
        right_up_leg = self.ru_body, self.ru_shape, self.ru_joint, self.ru_motor
        right_down_leg = self.rd_body, self.rd_shape, self.rd_joint, self.rd_motor
        right_foot = self.rf_body, self.rf_shape, self.rf_joint, self.rf_motor

        return body, head, left_arm, right_arm, left_up_leg, left_down_leg, left_foot, right_up_leg, right_down_leg, right_foot

    def get_data(self):
        lu = ((360 - math.degrees(self.lu_body.angle)) - (360 - math.degrees(self.body.angle))) / 360.0
        ld = ((360 - math.degrees(self.ld_body.angle)) - (360 - math.degrees(self.body.angle))) / 360.0
        lf = ((360 - math.degrees(self.lf_body.angle)) - (360 - math.degrees(self.body.angle))) / 360.0
        ru = ((360 - math.degrees(self.ru_body.angle)) - (360 - math.degrees(self.body.angle))) / 360.0
        rd = ((360 - math.degrees(self.rd_body.angle)) - (360 - math.degrees(self.body.angle))) / 360.0
        rf = ((360 - math.degrees(self.rf_body.angle)) - (360 - math.degrees(self.body.angle))) / 360.0
        la = ((360 - math.degrees(self.left_arm_upper_body.angle)) - (360 - math.degrees(self.body.angle))) / 360.0
        ra = ((360 - math.degrees(self.right_arm_upper_body.angle)) - (360 - math.degrees(self.body.angle))) / 360.0
        return self.body.angle, lu, ld, lf, la, ru, rd, rf, ra

    def draw_face(self, screen):
        if self.distance > 400:
            rotated_face = rot_center(self.face, math.degrees(self.head_body.angle))
        else:
            rotated_face = rot_center(self.red_face, math.degrees(self.head_body.angle))

        screen.blit(rotated_face, (self.head_body.position[0] - 50, screen_height - self.head_body.position[1] - 50))

    def set_color(self, color, rest_color = (0, 0, 255), shoe_color = (50, 50, 50)):
        self.shape.color = color
        self.head_shape.color = color
        self.left_arm_upper_shape.color = rest_color
        self.right_arm_upper_shape.color = rest_color
        self.lu_shape.color = rest_color
        self.ld_shape.color = rest_color
        self.lf_shape.color = shoe_color
        self.ru_shape.color = rest_color
        self.rd_shape.color = rest_color
        self.rf_shape.color = shoe_color

    def update(self):
        #lu
        self.lu_flag = False
        if (360 - math.degrees(self.lu_body.angle)) - (360 - math.degrees(self.body.angle)) >= 90 and self.lu_motor.rate > 0:
            self.lu_motor.rate = 0
            self.lu_flag = True
        elif (360 - math.degrees(self.lu_body.angle)) - (360 - math.degrees(self.body.angle)) <= -90 and self.lu_motor.rate < 0:
            self.lu_motor.rate = 0
            self.lu_flag = True

        #ld
        self.ld_flag = False
        if (360 - math.degrees(self.ld_body.angle)) - (360 - math.degrees(self.lu_body.angle)) >= 90 and self.ld_motor.rate > 0:
            self.ld_motor.rate = 0
            self.ld_flag = True
        elif (360 - math.degrees(self.ld_body.angle)) - (360 - math.degrees(self.lu_body.angle)) <= -90 and self.ld_motor.rate < 0:
            self.ld_motor.rate = 0
            self.ld_flag = True

        #ru
        self.ru_flag = False
        if (360 - math.degrees(self.ru_body.angle)) - (360 - math.degrees(self.body.angle)) >= 90 and self.ru_motor.rate > 0:
            self.ru_motor.rate = 0
            self.ru_flag = True
        elif (360 - math.degrees(self.ru_body.angle)) - (360 - math.degrees(self.body.angle)) <= -90 and self.ru_motor.rate < 0:
            self.ru_motor.rate = 0
            self.ru_flag = True

        #rd
        self.rd_flag = False
        if (360 - math.degrees(self.rd_body.angle)) - (360 - math.degrees(self.ru_body.angle)) >= 90 and self.rd_motor.rate > 0:
            self.rd_motor.rate = 0
            self.rd_flag = True
        elif (360 - math.degrees(self.rd_body.angle)) - (360 - math.degrees(self.ru_body.angle)) <= -90 and self.rd_motor.rate < 0:
            self.rd_motor.rate = 0
            self.rd_flag = True


        #lf
        self.lf_flag = False
        if (360 - math.degrees(self.lf_body.angle)) - (360 - math.degrees(self.ld_body.angle)) >= 90 and self.lf_motor.rate > 0:
            self.lf_motor.rate = 0
            self.lf_flag = True
        elif (360 - math.degrees(self.lf_body.angle)) - (360 - math.degrees(self.ld_body.angle)) <= -45 and self.lf_motor.rate < 0:
            self.lf_motor.rate = 0
            self.lf_flag = True


        #rf
        self.rf_flag = False
        if (360 - math.degrees(self.rf_body.angle)) - (360 - math.degrees(self.rd_body.angle)) >= 90 and self.rf_motor.rate > 0:
            self.rf_motor.rate = 0
            self.rf_flag = True
        elif (360 - math.degrees(self.rf_body.angle)) - (360 - math.degrees(self.rd_body.angle)) <= -45 and self.rf_motor.rate < 0:
            self.rf_motor.rate = 0
            self.rf_flag = True

        """
        #left arm
        self.la_flag = False
        if (360 - math.degrees(self.left_arm_upper_body.angle)) - (360 - math.degrees(self.body.angle)) >= 90 and self.la_motor.rate > 0:
            self.la_motor.rate = 0
            self.la_flag = True
        elif (360 - math.degrees(self.left_arm_upper_body.angle)) - (360 - math.degrees(self.body.angle)) <= -90 and self.la_motor.rate < 0:
            self.la_motor.rate = 0
            self.la_flag = True

        #right arm
        self.ra_flag = False
        if (360 - math.degrees(self.right_arm_upper_body.angle)) - (360 - math.degrees(self.body.angle)) >= 90 and self.ra_motor.rate > 0:
            self.ra_motor.rate = 0
            self.ra_flag = True
        elif (360 - math.degrees(self.right_arm_upper_body.angle)) - (360 - math.degrees(self.body.angle)) <= -90 and self.ra_motor.rate < 0:
            self.ra_motor.rate = 0
            self.ra_flag = True
        """

        if self.head_body.position.y <= 140 or self.distance > 1500 or self.tick > 120:
        #if self.distance > 1300 or self.tick > 120:
            self.is_done = True

        if self.distance <= 210:
            self.is_catch = True


    def add_space(self, space):
        space.add(self.body, self.shape, self.head_body, self.head_shape, self.head_joint)
        space.add(self.left_arm_upper_body, self.left_arm_upper_shape, self.left_arm_upper_joint, self.la_motor)
        space.add(self.right_arm_upper_body, self.right_arm_upper_shape, self.right_arm_upper_joint, self.ra_motor)
        space.add(self.lu_body, self.lu_shape, self.lu_joint, self.lu_motor)
        space.add(self.ru_body, self.ru_shape, self.ru_joint, self.ru_motor)
        space.add(self.ld_body, self.ld_shape, self.ld_joint, self.ld_motor)
        space.add(self.rd_body, self.rd_shape, self.rd_joint, self.rd_motor)
        space.add(self.lf_body, self.lf_shape, self.lf_joint, self.lf_motor)
        space.add(self.rf_body, self.rf_shape, self.rf_joint, self.rf_motor)

    def set_position(self, x):
        self.body._set_position((self.body.position.x - x, self.body.position.y))
        self.head_body._set_position((self.head_body.position.x - x, self.head_body.position.y))
        self.left_arm_upper_body._set_position((self.left_arm_upper_body.position.x - x, self.left_arm_upper_body.position.y))
        self.right_arm_upper_body._set_position((self.right_arm_upper_body.position.x - x, self.right_arm_upper_body.position.y))

        self.lu_body._set_position((self.lu_body.position.x - x, self.lu_body.position.y))
        self.ru_body._set_position((self.ru_body.position.x - x, self.ru_body.position.y))
        self.ld_body._set_position((self.ld_body.position.x - x, self.ld_body.position.y))
        self.rd_body._set_position((self.rd_body.position.x - x, self.rd_body.position.y))
        self.lf_body._set_position((self.lf_body.position.x - x, self.lf_body.position.y))
        self.rf_body._set_position((self.rf_body.position.x - x, self.rf_body.position.y))

class Human:
    def __init__(self):
        moment = pymunk.moment_for_box(100, (100, 30))
        self.body = pymunk.Body(100, moment)
        self.shape = pymunk.Poly.create_box(self.body, (100, 30))

        self.body.position = (1200, 200)
        self.shape.body = self.body
        self.shape.color = (150, 150, 0, 255)
        space.add(self.body, self.shape)

        wheel_moment = pymunk.moment_for_circle(100, 20, 25)
        self.wheel1_body = pymunk.Body(100, wheel_moment)
        self.wheel1_body.position = self.body.position + (-80, -20)
        self.wheel1_shape = pymunk.Circle(self.wheel1_body, 25)
        self.wheel1_shape.friction = 1.5
        self.wheel1_shape.color = (200, 200, 200)
        self.wheel1_joint = pymunk.PinJoint(self.body, self.wheel1_body, (-50, 0), (0, 0))
        self.wheel1_joint2 = pymunk.PinJoint(self.body, self.wheel1_body, (-50, -15), (0, 0))
        self.wheel1_motor = pymunk.SimpleMotor(self.wheel1_body, self.body, -5)
        space.add(self.wheel1_body, self.wheel1_shape, self.wheel1_joint, self.wheel1_joint2, self.wheel1_motor)


        wheel_moment = pymunk.moment_for_circle(100, 20, 25)
        self.wheel2_body = pymunk.Body(100, wheel_moment)
        self.wheel2_body.position = self.body.position + (80, -20)
        self.wheel2_shape = pymunk.Circle(self.wheel2_body, 25)
        self.wheel2_shape.friction = 1.5
        self.wheel2_shape.color = (200, 200, 200)
        self.wheel2_joint = pymunk.PinJoint(self.body, self.wheel2_body, (50, 0), (0, 0))
        self.wheel2_joint2 = pymunk.PinJoint(self.body, self.wheel2_body, (50, -15), (0, 0))
        self.wheel2_motor = pymunk.SimpleMotor(self.wheel2_body, self.body, -5)
        space.add(self.wheel2_body, self.wheel2_shape, self.wheel2_joint, self.wheel2_joint2, self.wheel2_motor)

        self.human = pygame.image.load('human.png')
        self.human = pygame.transform.scale(self.human, (100, 150))

        self.huk = pygame.image.load('huk.png')
        self.huk = pygame.transform.scale(self.huk, (100, 150))

        self.is_dead = False
        self.catch = 0

    def draw_human(self, screen, distance):
        if distance > 400:
            screen.blit(self.human, (self.body.position[0] - 60, screen_height - self.body.position[1] - 150))
        else:
            screen.blit(self.huk, (self.body.position[0] - 60, screen_height - self.body.position[1] - 150))

    def get_shapes(self):
        return self.body, self.shape, \
        self.wheel1_body, self.wheel1_shape, self.wheel1_joint, self.wheel1_joint2, self.wheel1_motor, \
        self.wheel2_body, self.wheel2_shape, self.wheel2_joint, self.wheel2_joint2, self.wheel2_motor

    def set_position(self, x):
        self.body._set_position((self.body.position.x - x, int(self.body.position.y)))
        self.wheel1_body._set_position((self.wheel1_body.position.x - x, int(self.wheel1_body.position.y)))
        self.wheel2_body._set_position((self.wheel2_body.position.x - x, int(self.wheel2_body.position.y)))

def add_land(space):
    body = pymunk.Body(body_type=pymunk.Body.STATIC)
    body.position = (0, 100)
    land = pymunk.Segment(body, (0, 0), (99999, 0), 10)
    land.friction = 0.5
    land.elasticity = 0.1
    space.add(land)

def rot_center(image, angle):
    orig_rect = image.get_rect()
    rot_image = pygame.transform.rotate(image, angle)
    rot_rect = orig_rect.copy()
    rot_rect.center = rot_image.get_rect().center
    rot_image = rot_image.subsurface(rot_rect).copy()
    return rot_image


def check_distance(robot, human):
    distance = math.sqrt(math.pow(robot[0] - human[0], 2) + math.pow(robot[1] - human[1], 2))
    return distance


def run_test():
    pygame.init()
    screen = pygame.display.set_mode((screen_width, screen_height))
    clock = pygame.time.Clock()
    draw_options = pymunk.pygame_util.DrawOptions(screen)
    font = pygame.font.SysFont("Arial", 30)


    robot = Robot()
    add_land(space)

    #main game
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit(0)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    robot.ru_motor.rate = 5
                elif event.key == pygame.K_DOWN:
                    robot.ru_motor.rate = -5
                if event.key == pygame.K_RIGHT:
                    robot.rd_motor.rate = 5
                elif event.key == pygame.K_LEFT:
                    robot.rd_motor.rate = -5

                if event.key == pygame.K_w:
                    robot.lu_motor.rate = 5
                elif event.key == pygame.K_s:
                    robot.lu_motor.rate = -5
                if event.key == pygame.K_a:
                    robot.ld_motor.rate = 5
                elif event.key == pygame.K_d:
                    robot.ld_motor.rate = -5

                if event.key == pygame.K_i:
                    robot.la_motor.rate = 5
                elif event.key == pygame.K_k:
                    robot.la_motor.rate = -5
                if event.key == pygame.K_j:
                    robot.ra_motor.rate = 5
                elif event.key == pygame.K_l:
                    robot.ra_motor.rate = -5

                if event.key == pygame.K_t:
                    robot.lf_motor.rate = 5
                elif event.key == pygame.K_g:
                    robot.lf_motor.rate = -5
                if event.key == pygame.K_f:
                    robot.rf_motor.rate = 5
                elif event.key == pygame.K_h:
                    robot.rf_motor.rate = -5

        #print(math.degrees(abs(robot.left_arm_upper_body.angle)) % 360)

        robot.update()
        space.step(1/50.0)
        screen.fill((255, 255, 255))
        space.debug_draw(draw_options)
        robot.draw_face(screen)
        pygame.display.flip()
        clock.tick(60)


        robot.ru_motor.rate = 0
        robot.rd_motor.rate = 0
        robot.lu_motor.rate = 0
        robot.ld_motor.rate = 0
        robot.la_motor.rate = 0
        robot.ra_motor.rate = 0
        robot.lf_motor.rate = 0
        robot.rf_motor.rate = 0


def run_human(genomes, config):
    pygame.init()
    screen = pygame.display.set_mode((screen_width, screen_height))
    clock = pygame.time.Clock()
    draw_options = pymunk.pygame_util.DrawOptions(screen)
    generation_font = pygame.font.SysFont("Arial", 70)
    font = pygame.font.SysFont("Arial", 30)

    ruler = 0

    nets = []
    ge = []
    robots = []
    human = Human()

    for id, g in genomes:
        net = neat.nn.FeedForwardNetwork.create(g, config)
        nets.append(net)
        robots.append(Robot())
        g.fitness = 0
        ge.append(g)

    add_land(space)

    #main game
    global generation
    generation += 1
    if generation > 1000:
        draw_flag = True
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit(0)


        if len(robots) == 0:
            space.remove(human.get_shapes())
            break

        for i, robot in enumerate(robots):
            output = nets[i].activate(robot.get_data())
            speed = 5
            #print(output)
            for i, out in enumerate(output):
                if out > 0.9:
                    if i == 0 and robot.lu_flag == False:
                        robot.lu_motor.rate = speed
                    elif i == 1 and robot.lu_flag == False:
                        robot.lu_motor.rate = -speed
                    elif i == 2 and robot.ru_flag == False:
                        robot.ru_motor.rate = speed
                    elif i == 3 and robot.ru_flag == False:
                        robot.ru_motor.rate = -speed
                    elif i == 4 and robot.ld_flag == False:
                        robot.ld_motor.rate = speed
                    elif i == 5 and robot.ld_flag == False:
                        robot.ld_motor.rate = -speed
                    elif i == 6 and robot.rd_flag == False:
                        robot.rd_motor.rate = speed
                    elif i == 7 and robot.rd_flag == False:
                        robot.rd_motor.rate = -speed
                    elif i == 8 and robot.la_flag == False:
                        robot.la_motor.rate = speed
                    elif i == 9 and robot.la_flag == False:
                        robot.la_motor.rate = -speed
                    elif i == 10 and robot.ra_flag == False:
                        robot.ra_motor.rate = speed
                    elif i == 11 and robot.ra_flag == False:
                        robot.ra_motor.rate = -speed
                    elif i == 12 and robot.lf_flag == False:
                        robot.lf_motor.rate = speed
                    elif i == 13 and robot.lf_flag == False:
                        robot.lf_motor.rate = -speed
                    elif i == 14 and robot.rf_flag == False:
                        robot.rf_motor.rate = speed
                    elif i == 15 and robot.rf_flag == False:
                        robot.rf_motor.rate = -speed

        # check
        min = 9999
        at = 0
        for i, robot in enumerate(robots):
            robot.set_color((240, 240, 240), (240, 240, 240), (240, 240, 240))
            distance = int(check_distance(robot.body.position, human.body.position))
            #if robot.body.position.y >= 250:
                #ge[i].fitness += 1

            if distance < robot.distance:
                ge[i].fitness += 1
                robot.tick = 0
            elif distance > robot.distance:
                robot.tick += 1
            robot.distance = distance
            if distance < min:
                min = distance
                at = i
            robot.update()

        robots[at].set_color((255, 0, 0))
        robot_position = robots[at].body.position

        if robots[at].body.position.x > screen_width / 2:
            d = int(robots[at].body.position.x - screen_width / 2)
            human.set_position(d)
            for robot in robots:
                robot.set_position(d)
            ruler -= 3


        space.step(1/50.0)
        screen.fill((255, 255, 255))
        space.debug_draw(draw_options)

        robots[at].draw_face(screen)
        human.draw_human(screen, min)

        text = generation_font.render("Generation : " + str(generation), True, (0, 0, 0))
        text_rect = text.get_rect()
        text_rect.center = (screen_width/2, screen_height/2 - 100)
        screen.blit(text, text_rect)

        #draw distance
        text = font.render(str(int(min)), True, (0, 0, 0))
        text_rect = text.get_rect()
        if human.body.position.x < 1900:
            text_rect.center = (human.body.position.x - 100, 700)
        else:
            text_rect.center = (1800, 700)
        screen.blit(text, text_rect)


        text = font.render("Catch : " + str(human.catch), True, (0, 0, 0))
        text_rect = text.get_rect()
        text_rect.center = (human.body.position.x, screen_height - human.body.position.y - 200)
        screen.blit(text, text_rect)

        #arrow
        pygame.draw.line(screen, (0, 0, 0), (robot_position[0], 750), (human.body.position[0], 750))
        pygame.draw.line(screen, (0, 0, 0), (robot_position[0], 750), (robot_position[0] + 30, 725))
        pygame.draw.line(screen, (0, 0, 0), (robot_position[0], 750), (robot_position[0] + 30, 775))
        pygame.draw.line(screen, (0, 0, 0), (human.body.position[0], 750), (human.body.position[0] - 30, 725))
        pygame.draw.line(screen, (0, 0, 0), (human.body.position[0], 750), (human.body.position[0] - 30, 775))

        for i in range(ruler, 1960, 100):
            pygame.draw.line(screen, (0, 0, 0), (i, screen_height - 100), (i, screen_height - 90))

        if ruler < 0:
            ruler += 100

        for i, robot in enumerate(robots):

            robot.lu_motor.rate = 0
            robot.ru_motor.rate = 0
            robot.ld_motor.rate = 0
            robot.rd_motor.rate = 0
            robot.la_motor.rate = 0
            robot.ra_motor.rate = 0
            robot.lf_motor.rate = 0
            robot.rf_motor.rate = 0

            if robot.is_catch:
                ge[i].fitness += 1000
                human.catch += 1
                space.remove(robot.get_shapes())
                robots.remove(robot)
                ge.pop(i)
            elif robot.is_done:
                space.remove(robot.get_shapes())
                robots.remove(robot)
                ge.pop(i)


        pygame.display.flip()
        clock.tick(60)

def run(config_path):
    config = neat.config.Config(neat.DefaultGenome, neat.DefaultReproduction,
                                neat.DefaultSpeciesSet, neat.DefaultStagnation, config_path)

    p = neat.Population(config)
    p.add_reporter(neat.StdOutReporter(True))
    stats = neat.StatisticsReporter()
    p.add_reporter(stats)

    generation = 0
    winner = p.run(run_human, 1000)


#run_test()

if __name__ == "__main__":
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, "config-feedforward.txt")
    run(config_path)
