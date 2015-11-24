class debug_draw:
    def __init__(self, model):
        self.model = model

    def draw_rect(ax, x, y, w, h):
        ax.add_patch(patches.Rectangle(
                                       (x, y),     # (x,y)
                                       w,          # width
                                       h,          # height
                                       fill=False
                                       )
                     )

    def draw_heightmap(ax, x, y):
        dx = np.abs(x[1] - x[0])
        for i in range(0, len(x)):
            top_n = np.floor(y[i]/dx)
            for n in range(0, int(top_n)):
                #for n in range(0, 4):
                self.draw_rect(ax, dx*i, n*dx, dx, dx)
            self.draw_rect(ax, dx*i, top_n*dx, dx, (y[i]-top_n*dx))

    def draw_leg(self, height, theta):
        """ draw a leg on the 2D terrain """
        end_point = (np.sin(theta)*constants.ARM_LENGTH, height-constants.ARM_LENGTH*np.cos(theta))
        plt.plot(end_point[0], end_point[1], 'ro', markersize=3)
        plt.plot([0, end_point[0]], [height, end_point[1]] , 'r-', markersize=3)

    def draw_legs_solution(self, heightmaps, eps=0.05):
        """ find a solution and draw the heightmap and the legs for all the four wheels"""
        min_height = [np.min(hmap[1]) for hmap in heightmaps]
        min_height = min(min_height)
        max_height =  [np.max(hmap[1]) for hmap in heightmaps]
        max_height = max(max_height)
        max_height = max(max_height, constants.ARM_LENGTH)
        heights = np.linspace(min_height, np.max(hmap[1])+constants.ARM_LENGTH, 100)
        heights = heights[::-1]
        thetas = []
        sol_found = False
        for height in heights:
            # test if it is possible to find a valid angle for every wheel
            thetas = np.array([self.model.find_theta(height, hmap[0], hmap[1], eps) for hmap in heightmaps])
            if np.alltrue(thetas > 0):
                # a solution for all the four wheel was found
                sol_found = True
                break
        fig = plt.figure(figsize=(10, 10))
        for i in range(0, 4):
            hmap = heightmaps[i]
            ax = fig.add_subplot(2,2, i+1)
            ax.set_title("leg {} theta: {} ".format(i, np.rad2deg(thetas[i])))
            ax.set_ylabel("height (m)")
            ax.axis([0, 0.6, 0, 0.6])
            self.draw_heightmap(ax, hmap[0], hmap[1])
            if sol_found > 0:
                self.draw_leg(height, thetas[i])
