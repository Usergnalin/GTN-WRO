import pygame
import pygame.gfxdraw
import os

class PlayfieldViewer:
    def __init__(self, screen_size, image_name):
        pygame.init()
        self.screen = pygame.display.set_mode(screen_size)
        pygame.display.set_caption("Playfield Viewer")
        self.clock = pygame.time.Clock()

        # Load image
        image_path = os.path.join("playfields", image_name)
        self.img = pygame.image.load(image_path).convert()
        self.img_w, self.img_h = self.img.get_size()

        screen_w, screen_h = self.screen.get_size()

        # Zoom and pan
        self.move_speed = 5000   # pixels per second
        self.zoom = 0.1
        self.zoom_step = 0.05
        self.max_zoom = 1.0
        self.min_zoom = 0.1
        self.offset_x = screen_w / 2 - (self.img_w * self.zoom) / 2
        self.offset_y = screen_h / 2 - (self.img_h * self.zoom) / 2
        self.dragging = False
        self.drag_start_pos = (0, 0)
        self.start_offset = (0, 0)

        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.drag_last_pos = None
        self.friction = 3.0   # higher = stops faster

        # Path elements
        self.path_elements = {
            "arrows": [
                {"start": (1000, 1000), "end": (2000, 1000)},
                {"start": (2000, 1000), "end": (3000, 1000)},
            ],
            "arcs": [
                {"center": (3000, 1000), "start_angle": 0, "end_angle": 3.14},
            ],
            "checkpoints": [
                {"pos": (1500, 1000)},
                {"pos": (3500, 1500)},
            ]
        }

    def zoom_towards_mouse(self, mouse_pos, old_zoom, new_zoom):
        mx, my = mouse_pos
        scale = new_zoom / old_zoom
        self.offset_x = mx - (mx - self.offset_x) * scale
        self.offset_y = my - (my - self.offset_y) * scale

    def clamp_offset(self):
        screen_w, screen_h = self.screen.get_size()

        cx_screen = screen_w / 2
        cy_screen = screen_h / 2

        min_offset_x = cx_screen - self.img_w * self.zoom
        max_offset_x = cx_screen

        min_offset_y = cy_screen - self.img_h * self.zoom
        max_offset_y = cy_screen

        self.offset_x = max(min_offset_x, min(max_offset_x, self.offset_x))
        self.offset_y = max(min_offset_y, min(max_offset_y, self.offset_y))



    def get_visible_image(self):
        inv_zoom = 1 / self.zoom
        left = max(0, int(-self.offset_x * inv_zoom))
        top = max(0, int(-self.offset_y * inv_zoom))
        right = min(self.img_w, int((-self.offset_x + self.screen.get_width()) * inv_zoom))
        bottom = min(self.img_h, int((-self.offset_y + self.screen.get_height()) * inv_zoom))

        cropped = self.img.subsurface(pygame.Rect(left, top, right - left, bottom - top))
        scaled_width = int((right - left) * self.zoom)
        scaled_height = int((bottom - top) * self.zoom)
        return pygame.transform.scale(cropped, (scaled_width, scaled_height)), left, top

    def transform(self, pos):
        x, y = pos
        return int(x * self.zoom + self.offset_x), int(y * self.zoom + self.offset_y)

    def update_keyboard_pan(self, dt):
        keys = pygame.key.get_pressed()

        speed = (self.move_speed * self.zoom ** 0.5) * dt

        if keys[pygame.K_a]:
            self.offset_x += speed
            self.clamp_offset()
        if keys[pygame.K_d]:
            self.offset_x -= speed
            self.clamp_offset()
        if keys[pygame.K_w]:
            self.offset_y += speed
            self.clamp_offset()
        if keys[pygame.K_s]:
            self.offset_y -= speed
            self.clamp_offset()


    def draw_path_elements(self):
        # Draw arrows
        for arrow in self.path_elements["arrows"]:
            start = self.transform(arrow["start"])
            end = self.transform(arrow["end"])
            pygame.gfxdraw.line(self.screen, start[0], start[1], end[0], end[1], (255, 0, 0))

        # Draw arcs
        for arc in self.path_elements["arcs"]:
            cx, cy = self.transform(arc["center"])
            radius = 100
            rect = pygame.Rect(cx - radius, cy - radius, radius * 2, radius * 2)
            pygame.draw.arc(self.screen, (0, 255, 0), rect, arc["start_angle"], arc["end_angle"], 2)

        # Draw checkpoints
        for cp in self.path_elements["checkpoints"]:
            pos = self.transform(cp["pos"])
            pygame.draw.circle(self.screen, (0, 0, 255), pos, 5)
    
    def update_momentum(self, dt):
        if self.dragging:
            return

        # apply velocity
        self.offset_x += self.velocity_x * 60 * dt
        self.offset_y += self.velocity_y * 60 * dt

        # friction
        decay = max(0.0, 1.0 - self.friction * dt)
        self.velocity_x *= decay
        self.velocity_y *= decay

        # stop when slow
        if abs(self.velocity_x) < 0.1:
            self.velocity_x = 0
        if abs(self.velocity_y) < 0.1:
            self.velocity_y = 0

        self.clamp_offset()

    def handle_event(self, event):
        if event.type == pygame.QUIT:
            return False

        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 2:  # Middle click drag
                self.dragging = True
                self.drag_start_pos = pygame.mouse.get_pos()
                self.start_offset = (self.offset_x, self.offset_y)

            elif event.button in (4, 5) and not self.dragging:  # Scroll
                old_zoom = self.zoom
                if event.button == 4:  # zoom in
                    self.zoom = min(self.max_zoom, self.zoom + self.zoom_step)
                else:  # zoom out
                    self.zoom = max(self.min_zoom, self.zoom - self.zoom_step)
                self.zoom_towards_mouse(pygame.mouse.get_pos(), old_zoom, self.zoom)
                
                
        elif event.type == pygame.MOUSEWHEEL:
            old_zoom = self.zoom

            if event.y > 0:
                self.zoom = min(self.max_zoom, self.zoom + self.zoom_step)
            elif event.y < 0:
                self.zoom = max(self.min_zoom, self.zoom - self.zoom_step)

            if self.zoom != old_zoom:
                self.zoom_towards_mouse(pygame.mouse.get_pos(), old_zoom, self.zoom)

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                screen_w, screen_h = self.screen.get_size()

                # reset zoom if you want
                self.zoom = self.min_zoom

                # stop movement
                self.velocity_x = 0
                self.velocity_y = 0

                # center image on screen
                self.offset_x = screen_w / 2 - (self.img_w * self.zoom) / 2
                self.offset_y = screen_h / 2 - (self.img_h * self.zoom) / 2

                self.clamp_offset()
            if event.key == pygame.K_ESCAPE:
                return False

        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 2:
                self.dragging = False
                self.drag_last_pos = None

        elif event.type == pygame.MOUSEMOTION and self.dragging:
            mx, my = pygame.mouse.get_pos()

            dx = mx - self.drag_start_pos[0]
            dy = my - self.drag_start_pos[1]

            self.offset_x = self.start_offset[0] + dx
            self.offset_y = self.start_offset[1] + dy

            # compute velocity (pixels per frame)
            if self.drag_last_pos is not None:
                self.velocity_x = event.rel[0]
                self.velocity_y = event.rel[1]

            self.drag_last_pos = (mx, my)
            self.clamp_offset()

        return True

    def run(self):
        running = True
        while running:
            for event in pygame.event.get():
                if not self.handle_event(event):
                    running = False

            self.screen.fill((0, 0, 0))
            visible_img, left, top = self.get_visible_image()
            blit_x = int(self.offset_x + left * self.zoom)
            blit_y = int(self.offset_y + top * self.zoom)
            self.screen.blit(visible_img, (blit_x, blit_y))
            self.draw_path_elements()
            pygame.display.flip()
            dt = self.clock.tick(144) / 1000.0
            self.update_keyboard_pan(dt)
            self.update_momentum(dt)

        pygame.quit()


if __name__ == "__main__":
    viewer = PlayfieldViewer((1920, 1080), "2024_senior.png")
    viewer.run()
