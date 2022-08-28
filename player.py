import sys

from src.simulator import Fleet, Simulator, Surface


surface = Surface()
# surface_n = int(input())  # the number of points used to draw the surface of Mars.
surface_n = 6
fake_input = [
    "0 1500",
    "1000 2000",
    "2000 500",
    "3500 500",
    "5000 1500",
    "6999 1000",
]


for i in range(surface_n):
    land_x, land_y = [int(j) for j in fake_input[i].split()]
    surface.add_point(land_x, land_y)
surface.calculation_landing_zone()
time_to_power = 0
turn = 0
powers = []
rotations = []
score = -1000

pop_size = 20
fleet = Fleet(
    surface=surface,
    nb_orders=60,
    fleet_size=pop_size,
    graded_rate=0.5,
    rand_rate=0.5,
    init_x=2500,
    init_y=2500,
    init_h_speed=0,
    init_v_speed=0,
    init_fuel=3000,
    init_power=0,
    init_rotation=0,
)
i_gen = 0
for i in range(100):
    # while fleet.generations[i_gen][0].score < 1000:
    fleet.new_generation(surface)
    i_gen += 1
    score = fleet.generations[i_gen][0].score
    if score > 1000:
        break
print(score, file=sys.stderr, flush=True)
powers = fleet.generations[i_gen][0].powers
rotations = fleet.generations[i_gen][0].rotations
