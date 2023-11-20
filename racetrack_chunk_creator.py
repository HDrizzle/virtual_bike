# Imports
import json, math, os

# initial stuff
tc = math.pi/180
size = 1000
grid_size = 100
bank_slope = 0.3
track_width = 100
map_name = 'racetrack'
precision = size / grid_size

def elev_distance_from_track_center(dist: float) -> float:
	if dist > size/4:
		return bank_slope * track_width
	else:
		return max((dist - (size/2 - track_width)) * bank_slope, 0)

def get_elev(x, y) -> float:
	half_size = size/2
	return ((x - half_size)**2 + (y - half_size)**2)**0.5 * bank_slope
	if y < size/2:# Inside track section
		dist = abs(y - size/4)
		# TODO
		#if not (size*0.25 < x < size*0.75):
		#	dist = 
		return elev_distance_from_track_center(dist)
	else:
		return 0

def save(grid) -> None:
	os.chdir(f'resources/maps/{map_name}/chunks/0_0')
	with open('data.json', 'w') as f:
		f.write(json.dumps({"position":[0.0,0.0,0.0],
			"ref_":{"position":[0,0]},
			"elevation":
			{
				"precision": precision,
				"grid": grid
			},
			"size": size,
			"grid_size": grid_size,
			"background_color":[0,128,0]
		}))

# Create grid
grid = []
for y_grid in range(grid_size + 1):
	curr_row = []
	for x_grid in range(grid_size + 1):
		x, y = (x_grid * precision, y_grid * precision)
		curr_row.append(get_elev(x, y))
	grid.append(curr_row)
# Save
save(grid)