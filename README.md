# Simple curb detection method

`curb_aggregate` requires a point cloud topic. It combines a few clouds together (using `tf` lookup) which makes the cloud denser and hence easier to process. It needs a config file (example in `config` folder)

There are two methods
- using normal and curvature threshold
- dividing a surrounding point cloud to an image like collection of cells, using height difference between min and max z in each cell to determine if the point in a cell belongs to the curb

Both methods are still quite noisy
