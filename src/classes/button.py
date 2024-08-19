import pygame

class Button:
    """This class is for creating Buttons with on-click functionality."""
    color: tuple[int, int, int]
    hover_color: tuple[int, int, int]
    x: tuple[int, int]
    y: tuple[int, int]
    width: int
    height: int
    text: str
    on_click: callable
    cargs: tuple

    def __init__(self, color: tuple[int, int, int], hover_color: tuple[int, int, int], start_point: tuple[int, int], dim: tuple[int, int], text: str, on_click: callable=None, cargs: tuple=()) -> None:
        self.color = color
        self.hover_color = hover_color
        self.x = start_point[0]
        self.y = start_point[1]
        self.width = dim[0]
        self.height = dim[1]
        self.text = text
        self.on_click = on_click
        self.cargs = cargs

    def is_selected(self) -> bool:
        """Return True if the mouse is on the button, else False"""
        mouse = pygame.mouse.get_pos()
        return self.x <= mouse[0] <= self.x + self.width and self.y <= mouse[1] <= self.y + self.height

    def click(self) -> None:
        """Runs self.on_click function if such a function exists and the button is selected"""
        if self.is_selected() and self.on_click:
            self.on_click(*self.cargs)
            
    def set_colour(self, color: tuple[int, int, int]) -> None:
        self.color = color