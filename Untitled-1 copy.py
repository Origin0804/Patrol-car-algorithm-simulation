import pygame
import math

# 初始化Pygame
pygame.init()
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

# 颜色常量
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
PAUSE_COLOR = (255, 165, 0)

class Track:
    def __init__(self):
        self.points = []
        self.thickness = 2 
        
    def point_on_track(self, point):
        if len(self.points) < 2:
            return False
        for i in range(len(self.points)-1):
            a = self.points[i]
            b = self.points[i+1]
            distance = self.point_to_line_distance(point, a, b)
            if distance < self.thickness/2:
                return True
        return False
    
    def point_to_line_distance(self, p, a, b):
        line_vec = (b[0]-a[0], b[1]-a[1])
        p_vec = (p[0]-a[0], p[1]-a[1])
        line_len_sq = line_vec[0]**2 + line_vec[1]**2
        if line_len_sq == 0:
            return math.hypot(p_vec[0], p_vec[1])
        t = max(0, min(1, (p_vec[0]*line_vec[0] + p_vec[1]*line_vec[1]) / line_len_sq))
        projection = (a[0] + t*line_vec[0], a[1] + t*line_vec[1])
        return math.hypot(p[0]-projection[0], p[1]-projection[1])
        
    def draw(self):
        if len(self.points) >= 2:
            pygame.draw.lines(screen, BLACK, False, self.points, self.thickness)

class LineFollower:
    def __init__(self, x, y):
        self.reset(x, y)
    
    def reset(self, x, y):
        self.x = x
        self.y = y
        self.angle = 0
        self.speed = 1#单位步长
        self.sensor_offset = [#五个传感器的相对位置
            (12, 5.3), (12, 2.1), (13.7, 0), 
            (12, -2.1), (12, -5.3)
        ]
        self.direction = 0
        self.turning = False
        self.start_angle = 0
        self.turn_progress = 0
        self.last_T_Actual = 0 

    def get_sensor_positions(self):
        # 添加角度旋转计算
        angle_rad = math.radians(self.angle)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        
        sensors = []
        for dx, dy in self.sensor_offset:
            # 旋转偏移量
            rotated_dx = dx * cos_a - dy * sin_a
            rotated_dy = dx * sin_a + dy * cos_a
            sensors.append((self.x + rotated_dx, self.y + rotated_dy))
        return sensors
    
    def draw(self):
        # 计算旋转后的车身四个顶点
        angle_rad = math.radians(self.angle)
        points = [
            (-30, -10),#原点为车前轴的中点，四个点的坐标是车的四个角的相对坐标
            (10, -10),
            (10, 10),
            (-30, 10)
        ]
        
        # 旋转所有顶点
        rotated_points = []
        for x, y in points:
            rotated_x = x * math.cos(angle_rad) - y * math.sin(angle_rad)
            rotated_y = x * math.sin(angle_rad) + y * math.cos(angle_rad)
            rotated_points.append((self.x + rotated_x, self.y + rotated_y))
        
        # 绘制旋转后的车身
        pygame.draw.polygon(screen, BLUE, rotated_points)
        
        # 绘制传感器（保持现有逻辑不变）
        for sensor in self.get_sensor_positions():
            color = GREEN if track.point_on_track(sensor) else RED
            pygame.draw.circle(screen, color, (int(sensor[0]), int(sensor[1])), 1)
def Turn90Degrees(car, direction):
    if not car.turning:
        car.start_angle = car.angle
        car.turning = True
        car.turn_progress = 0
        car.direction = direction


# PID 类实现
class PID:
    """PID控制器类"""
    def __init__(self, kp, ki, kd, setpoint, output_limits=(-45, 45)):
        self.kp = kp  # 比例系数
        self.ki = ki  # 积分系数
        self.kd = kd  # 微分系数
        self.setpoint = setpoint  # 目标值
        self.output_limits = output_limits  # 输出限制
        self.integral = 0  # 积分项
        self.prev_error = 0  # 上一次误差

    def compute(self, current_value):
        """计算PID输出"""
        error = self.setpoint - current_value
        self.integral += error
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(self.output_limits[0], min(self.output_limits[1], output))
        self.prev_error = error
        return output

# PID 参数
TKp = 1  # 比例系数
TKi = 0.0  # 积分系数
TKd = 1  # 微分系数
T_Target = 0  # 目标值

# 创建 PID 对象
pid = PID(TKp, TKi, TKd, T_Target)

def control_algorithm(car, track):
    """控制算法，用于控制巡线车的运动"""
    sensors = [track.point_on_track(p) for p in car.get_sensor_positions()]
    s1, s2, s3, s4, s5 = sensors

    T_Actual = None
    if s1:
        T_Actual = 0
    elif s2:
        T_Actual = 2
    elif s3:
        T_Actual = 3
    elif s4:
        T_Actual = 4
    elif s5:
        T_Actual = 6

    if T_Actual is not None:
        T_Actual -= 3
        car.last_T_Actual = T_Actual
    else:
        T_Actual = car.last_T_Actual

    T_Out = pid.compute(T_Actual)

    car.angle += min(T_Out, 2)


    car.x += car.speed * math.cos(math.radians(car.angle))
    car.y += car.speed * math.sin(math.radians(car.angle))



def control_algorithm2(car, track):
    """
    控制算法函数，根据传感器数据控制小车的移动和转向。

    参数:
    car (LineFollower): 巡线小车对象。
    track (Track): 巡线轨道对象。
    """
    sensors = [track.point_on_track(p) for p in car.get_sensor_positions()]
    s1, s2, s3, s4, s5 = sensors

    if car.turning:
        if car.turn_progress < 90:
            turn_step = 2 #每前进一个单位步长，转过的角度
            car.angle = (car.start_angle + car.direction * car.turn_progress) % 360
            car.turn_progress += turn_step
            car.x += car.speed * math.cos(math.radians(car.angle))
            car.y += car.speed * math.sin(math.radians(car.angle))
        else:
            car.turning = False
    else:
        if not s2 and not s3 and not s4:
            if car.direction == 0:
                pass
        elif s1 and s2 and s3 and not s4 and not s5:
            Turn90Degrees(car, 1)
        elif not s1 and not s2 and s3 and s4 and s5:
            Turn90Degrees(car, -1)
        elif s2 and not s3:
            car.direction = -1
            car.angle += 3
        elif s4 and not s3:
            car.direction = 1
            car.angle -= 3
        elif s3:
            car.angle += car.direction * 1.5

        if not car.turning:
            car.x += car.speed * math.cos(math.radians(car.angle))
            car.y += car.speed * math.sin(math.radians(car.angle))

# 初始化对象
track = Track()
car = LineFollower(400, 300)
paused = 1
pause_rect = pygame.Rect(10, 10, 80, 30)
reset_rect = pygame.Rect(100, 10, 80, 30)

# 主循环
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if pause_rect.collidepoint(event.pos):
                paused = not paused
            elif reset_rect.collidepoint(event.pos):
                car.reset(400, 300)
            else:
                track.points.append(pygame.mouse.get_pos())
    
    screen.fill(WHITE)
    
    pygame.draw.rect(screen, PAUSE_COLOR, pause_rect)
    pygame.draw.rect(screen, GREEN, reset_rect)
    font = pygame.font.SysFont(None, 24)
    screen.blit(font.render('Paused' if paused else 'Running', True, WHITE), (15, 15))
    screen.blit(font.render('Reset', True, WHITE), (105, 15))
    
    track.draw()
    if not paused:
        control_algorithm(car, track)
    car.draw()
    
    pygame.display.flip()
    clock.tick(60)

pygame.quit()
