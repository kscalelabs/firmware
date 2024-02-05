#!/usr/bin/env python
"""Runs the GUI in interactive mode."""

import pygame
import sys


def main() -> None:
    # Initialize Pygame
    pygame.init()

    # Set fullscreen display mode
    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)

    # Colors
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)

    # Button dimensions and positions
    button1_rect = pygame.Rect(50, 100, 200, 50)
    button2_rect = pygame.Rect(50, 200, 200, 50)

    def draw_button(rect, color, text):
        pygame.draw.rect(screen, color, rect)
        font = pygame.font.Font(None, 36)
        text_surf = font.render(text, True, BLACK)
        text_rect = text_surf.get_rect(center=rect.center)
        screen.blit(text_surf, text_rect)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if button1_rect.collidepoint(event.pos):
                    print("Button 1 clicked")
                    # Implement your action here
                elif button2_rect.collidepoint(event.pos):
                    print("Button 2 clicked")
                    # Implement your action here

        screen.fill(WHITE)
        draw_button(button1_rect, RED, "Button 1")
        draw_button(button2_rect, GREEN, "Button 2")

        # Update the display
        pygame.display.flip()

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    # python -m firmware.gui.interact
    main()
