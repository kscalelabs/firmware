#!/usr/bin/env python
"""Runs the GUI in interactive mode.

This is a GUI to run on the Raspberry Pi.
"""

import pygame
import sys

Color = tuple[int, int, int]

WHITE: Color = (255, 255, 255)
BLACK: Color = (0, 0, 0)
RED: Color = (255, 0, 0)
GREEN: Color = (0, 255, 0)
BLUE: Color = (0, 0, 255)


class Button:
    def __init__(self, rect: pygame.Rect, color: Color, text: str) -> None:
        self.rect = rect
        self.color = color
        self.text = text

    def draw(self, screen: pygame.Surface) -> None:
        pygame.draw.rect(screen, self.color, self.rect)
        font = pygame.font.Font(None, 36)
        text_surf = font.render(self.text, True, BLACK)
        text_rect = text_surf.get_rect(center=self.rect.center)
        screen.blit(text_surf, text_rect)

    def clicked(self, pos: tuple[int, int]) -> bool:
        return self.rect.collidepoint(pos)


def main() -> None:
    pygame.init()

    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)

    close_button = Button(pygame.Rect(screen.get_width() - 100, 0, 100, 50), RED, "Close")


    def draw_screen() -> None:
        screen.fill(WHITE)
        close_button.draw(screen)

    # Run the main loop.
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if close_button.clicked(event.pos):
                    running = False

        draw_screen()
        pygame.display.flip()

    # Clean up.
    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    # python -m firmware.gui.interact
    main()
