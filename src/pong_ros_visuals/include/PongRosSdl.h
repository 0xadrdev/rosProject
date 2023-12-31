#ifndef PONG_ROS_SDLH_
#define PONG_ROS_SDLH_

#include <cstdint>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h> // If this can not be found, install it with:  sudo apt install libsdl2-ttf-dev

/**
 * Description of SDL2_UI class
 */
class PongRosSdl
{
  public:
    const int sizeX; 
    const int sizeY;

    /// Constructor initializes the SDL2_UI fully.
    ///
    /// Creates a main (SDL) window for rendering.
    ///
    /// \param Dimensions (pixels) of the window to be created.
    PongRosSdl(std::string title, int sizeX, int sizeY, int fontSize);

      // SDL2_UI objects should not be copied or moved.
    PongRosSdl(const PongRosSdl&) = delete;
    PongRosSdl(const PongRosSdl&&) = delete;
    
    PongRosSdl &operator=(const PongRosSdl &) = delete;

    /// Destructor fully de-initializes the SDL2_UI, including closing the main window.
    virtual ~PongRosSdl();

    /// Clears the draw buffer (black).
    void clearWindow();

    /// Presents the draw buffer to the screen.
    void render();

    /// Draw pixel
    void setPixel(int x, int y);

    /// Draw filled rectangle
    void setRectangle(int x1, int y1, int x2, int y2);

    /// Draw text. Font and font size are fixed (change in SDL2_UI.cpp if needed).
    /// Text is always white.
    void setText(std::string s, int x_center, int y_center);

    /// Set the foreground color for the next drawing action
    void setColor(uint8_t r, uint8_t g, uint8_t b, uint8_t a);

    bool handleWindowEvents();

  private:
    /// Main SDL2_UI window.
    SDL_Window *window;
    /// SDL Renderer to draw items onto #window.
    SDL_Renderer *renderer;
    
    // Variables for text rendering
    TTF_Font *theFont;
    int blockiness; /// To mimic old-fashioned computer, we make the letters more blocky.

    void InitializeFontStuff(int fontSize);
    void DestroyFontStuff();
  };
#endif /* _SDL2SRC_UI_H_ */
