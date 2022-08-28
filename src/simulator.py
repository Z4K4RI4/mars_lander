# %%
import sys
import math
import numpy as np

# Auto-generated code below aims at helping you parse
# the standard input according to the problem statement.
G = 3.711  # gravity
turn = 0


def interpolation(x1, y1, x2, y2, x):
    if x2 == x1:
        return y1
    # with d like [[x1,y1], [x2,y2]]
    output = y1 + (x - x1) * ((y2 - y1) / (x2 - x1))
    return output


class Point:
    """Class defining a point in a 2D space
    coordinates are defined by x and y properties

    Returns:
        _type_: _description_
    """

    x: int = 0
    y: float = 0.0

    def __init__(self, x=0, y=0) -> None:
        self.x = float(x)
        self.y = float(y)

    def __str__(self) -> str:
        return "(" + str(self.x) + " " + str(self.y) + ")"


class Surface:
    """Class defining the surface as a 2D-line

    Raises:
        ValueError: an incorrect x value has been given, its altitude cannot be given
    """

    ground_points: list[Point] = []
    landing_zone: tuple([Point, Point])

    def add_point(self, x, y):
        """add a point when given a x and a y

        Args:
            x (int): absice value
            y (int): ordinate value
        """
        self.ground_points.append(Point(x, y))

    def ground_alt(self, a):
        """given the value of an x, finds the list of y corresponding in the line.

        Args:
            a (int|float): x where altitude is to be find

        Raises:
            ValueError: x is not in the range between max x and min x
        """
        list_alt = []
        previous_point = self.ground_points[0]
        if a < 0:
            list_alt.append(self.ground_points[0].y)
        if a >= 7000:
            list_alt.append(self.ground_points[-1].y)

        if a == 0:
            list_alt.append(previous_point.y)
        for point in self.ground_points[1:]:
            if a == point.x:
                list_alt.append(point.y)
            if previous_point.x < a < point.x:
                list_alt.append(
                    interpolation(
                        previous_point.x, previous_point.y, point.x, point.y, a
                    )
                )
            previous_point = point
        if list_alt == []:
            raise ValueError(
                "No altitude found for  x = " + str(a) + " check the value of a"
            )
        return list_alt

    def is_position_allowed(self, point: Point):
        """Tells if the position is in the area where the lander is allowed to navigate

        Args:
            point (Point): point investigated

        Returns:
            _type_: bool
        """
        list_alt = self.ground_alt(point.x)
        down_altitudes = [a for a in list_alt if a < point.y]
        if len(down_altitudes) % 2 == 0:
            return False
        else:
            return True

    def calculation_landing_zone(self):
        """Finds the landing zone in the line
        landing zone has to be horizontal with a length of at least 1000

        Returns:
            _type_: tuple(Point,Point)
        """
        previous_point = self.ground_points[0]
        for point in self.ground_points[1:]:
            if point.y == previous_point.y:
                if point.x - previous_point.x >= 1000:
                    self.landing_zone = (previous_point, point)
                    return self.landing_zone
            previous_point = point


class Lander:
    """Class to define the comportement of a lander
    it is defined with its position, speeds, fuel, angle (from the horizontal, retrograd) and thrust power
    """

    position: Point
    angle: int
    power: int
    fuel: int
    h_speed: int
    v_speed: int
    h_acceleration: float
    v_acceleration: float
    vect_hist = []
    angle_hist = []
    fuel_history = []

    def __init__(
        self,
        x: int,
        y: int,
        h_speed: int,
        v_speed: int,
        fuel: int,
        rotate: int,
        power: int,
    ) -> None:
        """Create a lander with initials parameters

        Args:
            x (int): x position of the lander
            y (int): y position of the lander
            h_speed (int): horizontal speed
            v_speed (int): vertical speed
            fuel (int): fuel quantity
            rotate (int): angle of the lander from the horizontal in the retrograd way
            power (int): thrust power of the lander
        """
        self.position = Point(x, y)
        self.h_speed = h_speed
        self.v_speed = v_speed
        self.fuel = fuel
        self.init_fuel = fuel
        self.angle = rotate
        self.power = power

        self.vect_hist = [(x, y, h_speed, v_speed)]
        # self.fuel_history = [fuel]
        self.angle_hist = [rotate]

    def __compute(self):
        """integrate the acceleration computed from angle and power to find speed and position at next step"""
        self.h_acceleration = -self.power * math.sin(math.radians(self.angle))
        self.v_acceleration = (self.power * math.cos(math.radians(self.angle))) - G

        ### integral 1 -> speed
        self.h_speed += int(self.h_acceleration)
        self.v_speed += int(self.v_acceleration)
        ### integral 2 -> position
        x = self.position.x
        y = self.position.y
        x += int(self.h_speed + (self.h_acceleration / 2))
        y += int(self.v_speed + (self.v_acceleration / 2))
        self.position = Point(x, y)

        self.vect_hist.append((x, y, self.h_speed, self.v_speed))
        self.angle_hist.append(self.angle)
        # self.fuel_history.append(self.fuel)

    def next_position(self, thrust: int, rotate: int):
        """given the new orders, return the next position

        Args:
            thrust (int): thrust order
            rotate (int): rotation order

        Returns:
            tuple(int, int, int, int): tuple describing the speed vector : (x,y,h_speed,v_speed)
        """

        if thrust < 0:
            thrust = 0
        if thrust > 4:
            thrust = 4
        if abs(self.power - thrust) > 1:
            if self.power < thrust:
                thrust = min(self.power + 1, 4)
            else:
                thrust = max(self.power - 1, 0)
        if rotate > 90:
            rotate = 90
        if rotate < -90:
            rotate = -90

        if abs(self.angle - rotate) > 15:
            if self.angle < rotate:
                rotate = min(self.angle + 15, 90)
            else:
                rotate = max(self.angle - 15, -90)

        self.power = thrust
        self.angle = rotate
        self.fuel -= self.power
        self.__compute()
        return self.vect_hist[-1]


class Simulator(Lander):
    powers: list[int]
    rotations: list[int]
    length: int
    score: int = 0

    def __init__(
        self,
        x: int,
        y: int,
        h_speed: int,
        v_speed: int,
        fuel: int,
        rotate: int,
        power: int,
        power_list: list[int],
        rotation_list: list[int],
    ) -> None:
        super().__init__(x, y, h_speed, v_speed, fuel, rotate, power)
        assert len(power_list) == len(rotation_list)
        self.powers = power_list
        self.rotations = rotation_list
        self.length = len(power_list)

    def reset(self):
        self.position = Point(self.vect_hist[0][0], self.vect_hist[0][1])
        self.h_speed = self.vect_hist[0][2]
        self.v_speed = self.vect_hist[0][3]
        self.fuel = self.init_fuel
        self.angle = self.angle_hist[0]
        self.power = 0

        self.vect_hist = []
        self.angle_hist = []

    def land_a_lander(self, surf: Surface):
        for i in range(self.length):
            new_power = self.powers[i]
            new_rotation = self.rotations[i]

            if abs(new_power - self.power) > 1:
                if new_power > self.power:
                    new_power = self.power + 1
                else:
                    new_power = self.power - 1
            if abs(new_rotation - self.angle) > 15:
                if new_rotation < self.angle:
                    new_rotation = self.angle - 15
                else:
                    new_rotation = self.angle + 15
            self.next_position(new_power, new_rotation)
            # self.vector_hist.append(self.vect_hist[-1])
            if not surf.is_position_allowed(self.position):
                ### a interpoler ? ###
                vector_to_score = self.vect_hist[-2]
                self.score = self.score_calculation(
                    surf,
                    position_to_score=vector_to_score[0],
                    h_speed=vector_to_score[2],
                    v_speed=vector_to_score[3],
                    rotation=self.angle_hist[-2],
                    fuel=self.fuel,
                    init_fuel=self.init_fuel,
                )
                break
            self.score = -1000
        return float(self.score)

    def score_calculation(
        self,
        surf: Surface,
        position_to_score: Point,
        h_speed: int,
        v_speed: int,
        rotation: int,
        fuel: int,
        init_fuel: int,
    ):
        ####################    MODIFY  VARIABLES NAMES ###############
        score = 0
        x = position_to_score
        if not (surf.landing_zone[0].x < x < surf.landing_zone[1].x):
            center_zone = int((surf.landing_zone[0].x + surf.landing_zone[1].x) / 2)
            dist_penality = 100 - abs(x - center_zone)
            score = dist_penality
        elif abs(h_speed) > 20:
            h_speed_penality = abs(h_speed)
            score = 200 - h_speed_penality
        elif v_speed < -40:
            v_speed_penality = abs(v_speed)
            score = 300 - v_speed_penality
        elif rotation != 0:
            rot_penality = abs(rotation)
            score = 400 - rot_penality
        elif fuel < 0:
            score = 1000 - (100 * (fuel / init_fuel))
        else:
            score = 1500 - (100 * (fuel / init_fuel))

        return float(score)


class Fleet:
    """Population control for all landers"""

    sim_list: list[Simulator]
    generations: list[list[Simulator]]
    fleet_size: int
    nb_orders: int = 50
    graded_rate = 0.3
    rand_rate = 0.2
    selection = []

    def generate_orders(self, init_power: int = 0, init_rotation: int = 0):
        """generate random orders for the 1st iteration. Next power is alwars between previous +/- 1 and rotation between previous +/- 15

        Args:
            init_power (int, optional): first power order. Defaults to 0.
            init_rotation (int, optional): first rotation order. Defaults to 0.

        Returns:
            _type_: _description_
        """
        assert 0 <= init_power <= 4
        assert -90 <= init_rotation <= 90
        power_list = []
        rotation_list = []
        previous_power = init_power
        previous_rotation = init_rotation
        for i in range(self.nb_orders):
            low_power = max([previous_power - 1, 0])
            high_power = min([previous_power + 1, 4])
            power = np.random.randint(low=low_power, high=1 + high_power)
            low_rotation = max([previous_rotation - 15, -90])
            high_rotation = min([previous_rotation + 15, 90])
            rotation = np.random.randint(low=low_rotation, high=high_rotation + 1)
            power_list.append(power)
            rotation_list.append(rotation)
            previous_power = power
            previous_rotation = rotation
        return (power_list, rotation_list)

    def __init__(
        self,
        surface: Surface,
        nb_orders: int = 50,
        fleet_size: int = 10,
        graded_rate: int = 0.6,
        rand_rate: int = 0.4,
        init_x: int = 2500,
        init_y: int = 2500,
        init_h_speed: int = 0,
        init_v_speed: int = 0,
        init_fuel: int = 0,
        init_power: int = 0,
        init_rotation: int = 0,
    ) -> None:
        """Population control class. initialization performs the first scorring of the random population created.

        Args:
            surface (Surface): surface given by the exercise
            nb_orders (int, optional): length of the orders list generated. Defaults to 50.
            fleet_size (int, optional): size of the population. Defaults to 10.
            graded_rate (int, optional): proportion of graded we want to keep. Defaults to 0.6.
            rand_rate (int, optional): proportion of random landers we keep. Defaults to 0.4.
            init_x (int, optional): starting x. Defaults to 2500.
            init_y (int, optional): starting altitude. Defaults to 2500.
            init_h_speed (int, optional): starting horizontal speed. Defaults to 0.
            init_v_speed (int, optional): starting vertical speed. Defaults to 0.
            init_fuel (int, optional): starting fuel capacity. Defaults to 0.
            init_power (int, optional): starting power thrust. Defaults to 0.
            init_rotation (int, optional): starting rotation. Defaults to 0.
        """

        self.nb_orders = nb_orders
        self.fleet_size = fleet_size
        self.graded_rate = graded_rate / (graded_rate + rand_rate)  # normalization
        self.rand_rate = rand_rate / (graded_rate + rand_rate)  # normalization

        self.generations = []

        self.init_x = init_x
        self.init_y = init_y
        self.init_h_speed = init_h_speed
        self.init_v_speed = init_v_speed
        self.init_fuel = init_fuel
        self.init_power = init_power
        self.init_rotation = init_rotation

        # generation of fleet
        self.sim_list = []
        for i in range(self.fleet_size):
            power_list, rotation_list = self.generate_orders(init_power, init_rotation)
            new_sim = Simulator(
                x=self.init_x,
                y=self.init_y,
                h_speed=self.init_h_speed,
                v_speed=self.init_v_speed,
                fuel=self.init_fuel,
                rotate=self.init_rotation,
                power=self.init_power,
                power_list=power_list,
                rotation_list=rotation_list,
            )
            new_sim.land_a_lander(surface)
            self.sim_list.append(new_sim)
        self.sim_list.sort(key=Fleet.get_score)
        self.sim_list.reverse()
        self.generations.append(self.sim_list)

    def get_score(sim_list_elt):
        return sim_list_elt.score

    def select_graded(self):
        self.selection: list[Simulator] = []
        size_graded = int((self.graded_rate * self.fleet_size) // 2)
        # for i in range(size_graded):
        #     self.selection.append(self.generations[-1][-(i + 1)])
        size_rand = int((self.fleet_size * self.rand_rate) // 2)
        self.selection = self.generations[-1][:size_graded]
        # for _ in range(size_rand):
        rand_parent = np.random.choice(
            self.generations[-1][size_graded:], size=size_rand, replace=False
        ).tolist()
        self.selection += rand_parent

    def crossover(self):
        """for each selected parent, select a random other parent and a random index to crossover 2 childs"""
        childs = []
        for idx_parent, parent1 in enumerate(self.selection[:-1]):
            # selection of a random second parent
            parent2 = self.selection[idx_parent + 1]
            # parent2 = np.random.choice(self.selection, size=1)[0]
            # while self.selection.index(parent1) != self.selection.index(parent2):
            #     parent2 = np.random.choice(self.selection, size=1)[0]
            rand_cross_factor = np.random.random() / 2
            if idx_parent > len(self.selection) // 2:
                rand_cross_factor = 0.5 + (
                    np.random.random() / 2
                )  # generate a random cross_factor
            child1_powers = [None] * len(parent1.powers)
            child1_rotations = [None] * len(parent1.powers)
            child2_powers = [None] * len(parent1.powers)
            child2_rotations = [None] * len(parent1.powers)

            for i in range(len(parent1.powers)):
                mutation_rate = 0.1
                child1_powers[i] = int(
                    (parent1.powers[i] * rand_cross_factor)
                    + ((1 - rand_cross_factor) * parent2.powers[i])
                )
                if np.random.random() < mutation_rate:
                    child1_powers[i] = np.random.randint(low=0, high=5)

                child1_rotations[i] = int(
                    (parent1.rotations[i] * rand_cross_factor)
                    + ((1 - rand_cross_factor) * parent2.rotations[i])
                )
                if np.random.random() < mutation_rate:
                    child1_rotations[i] = np.random.randint(low=-90, high=91)

                child2_powers[i] = int(
                    (parent2.powers[i] * rand_cross_factor)
                    + ((1 - rand_cross_factor) * parent1.powers[i])
                )
                if np.random.random() < mutation_rate:
                    child1_powers[i] = np.random.randint(low=0, high=5)
                child2_rotations[i] = int(
                    (parent2.rotations[i] * rand_cross_factor)
                    + ((1 - rand_cross_factor) * parent1.rotations[i])
                )
                if np.random.random() < mutation_rate:
                    child2_rotations[i] = np.random.randint(low=-90, high=91)

            child1 = Simulator(
                x=self.init_x,
                y=self.init_y,
                h_speed=self.init_h_speed,
                v_speed=self.init_v_speed,
                fuel=self.init_fuel,
                rotate=self.init_rotation,
                power=self.init_power,
                power_list=child1_powers,
                rotation_list=child1_rotations,
            )
            child2 = Simulator(
                x=self.init_x,
                y=self.init_y,
                h_speed=self.init_h_speed,
                v_speed=self.init_v_speed,
                fuel=self.init_fuel,
                rotate=self.init_rotation,
                power=self.init_power,
                power_list=child2_powers,
                rotation_list=child2_rotations,
            )
            childs.append(child1)
            childs.append(child2)
        self.generations.append(childs)

    def mutation(self, flip_rate=60):
        """for each order, if flip_coin is grater than flip_rate, performs a mutation fo the order

        Args:
            flip_rate (int, optional): _description_. Defaults to 60.
        """
        for sim in self.generations[-1]:
            rand_genes = np.random.choice(
                list(range(sim.length)[1:-1]),
                int(flip_rate * sim.length // 100),
                replace=True,
            )
            for i in rand_genes:  # range(sim.length)[1:-1]:
                flip_coin = 100  # np.random.randint(low=0, high=101)
                if flip_coin > flip_rate:
                    min_order_power = min([sim.powers[i - 1], sim.powers[i + 1]])
                    max_order_power = max(sim.powers[i - 1], sim.powers[i + 1])
                    lowest_power = max(max_order_power - 1, 0)
                    highest_power = min(min_order_power + 1, 4)
                    if lowest_power < highest_power:
                        new_power = np.random.randint(
                            low=lowest_power, high=highest_power + 1
                        )
                    else:
                        new_power = np.random.randint(
                            low=highest_power, high=lowest_power + 1
                        )
                    sim.powers[i] = new_power
                flip_coin = np.random.randint(low=0, high=101)
                if flip_coin > flip_rate:
                    min_order_rotation = min(sim.rotations[i - 1], sim.rotations[i + 1])
                    max_order_rotation = max(sim.rotations[i - 1], sim.rotations[i + 1])
                    lowest_rotation = max(max_order_rotation - 15, -90)
                    highest_rotation = min(min_order_rotation + 15, 90)
                    if lowest_rotation < highest_rotation:
                        new_rotation = np.random.randint(
                            low=lowest_rotation, high=highest_rotation + 1
                        )
                    else:
                        new_rotation = np.random.randint(
                            low=highest_rotation, high=lowest_rotation + 1
                        )
                    # except ValueError:
                    #     raise ValueError('\t'.join([str(mx) for mx in [sim.rotations[i-1], sim.rotations[i], sim.rotations[i+1], lowest_rotation, highest_rotation]]))
                    sim.rotations[i] = new_rotation

    def new_generation(self, surface):
        ################# select, crossover, mutate, score, and maybe repeat
        self.select_graded()
        self.crossover()
        self.mutation(flip_rate=99)

        for sim in self.generations[-1]:
            Simulator.land_a_lander(sim, surface)
        self.generations[-1].sort(key=Fleet.get_score)
        self.generations[-1].reverse()


# %%
