1. find blobs in each frame, each blob is idealy a vehicle
- subtract 2 consecutive frames to get foreground 
- threshold to get binary image
- dilate and erode to connect discrete parts
- find contours
- find convex hulls
- convert convert hulls to blobs (only keep blobs that meet certain criterias)
- using car move prediction algorithm to predict next position of a vehicle.
Calculate the distance between this predicted position to actually position. If the
distance is small enough, conclude this blob has already existed, simply replace the current blob with the new blob (update ceter position). If the distance is big, it suggests the new blob has not existed yet, simply add it the list of existing blobs.
- In order to do so, we need to keep a list of the center positions of each blob over time

2. when each blob passes a horizontal line, vehicle count increasess 
- repear those step with 2 consecutive frames until the video ends

3. This works well under scenariors
- have separate cars coming 
- cars coming together, harder to distinquish them--> inaccurate counting
