# Writeup

## Starter code

These scripts contain a basic planning implementation that includes an a-star search optimized with collinearity performed on a grid environment.

Argument | Value | Description
--- | --- | ---
--tlon | float | Global goal longitude.
--tlat | float | Global goal latitude.
--falt | float | Flight altitude

Please refer to the help command `./motion_planning.py -h` for more details.

## Path Planning Algorithm

### Home position

Home position from the coordinates provided with the first line of the `colliders.csv`.
The code is position and separator agnostic. It requires coordinate values provided 
for longitude and latitude. For providing longitude, type `lon0` and value separated with a space. 
For providing latitude, type `lat0` and value separated with a space.

Home position altitude will always be hardcoded to `0`.


### Local postion

We determine local position by converting out global position relative to the global home position.
`global_to_local(...)` is used for getting local postion.


### Grid position

Once the grid is created from the `colliders.csv` data, we calculate the grid start and end positions.

Grid start position is calculated by adding north and east offsets to the local position.

Grid end position could be determined in 2 ways. If `--tlon` and `--tlat` arguments are provided, given 
coordinates will be converted to local using `global_to_local(...)` and adding north and east offsets to 
the result. Otherwise, a random grid cell without an obstacle will be selected.

### Path search

A* search is performed on a grid using sideways (i.e., EAST, WEST, NORTH, SOUTH) and diagonal 
(i.e., NORTH_EAST, NORTH_WEST, SOUTH_EAST, SOUTH_WEST) directions.

Sideway movements cost less than diagonal movements. A sideway movement costs 1, while a diagonal movement 
costs square root of 2 (~1.41).

### Search optimization

If a valid path is found, the coordinates will be optimized with a collinearity test. We will remove 
any mid-point that is along the same trajectory with prior and post points.
