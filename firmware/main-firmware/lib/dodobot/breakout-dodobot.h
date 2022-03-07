#ifndef __DODOBOT_BREAKOUT_H__
#define __DODOBOT_BREAKOUT_H__

#include <Arduino.h>
#include <math.h>

#include "dodobot.h"
#include "display-dodobot.h"


namespace dodobot_breakout
{
    const int16_t X_MIN = 0;
    const int16_t X_MAX = ST7735_TFTHEIGHT_160;
    const int16_t Y_MIN = 0;
    const int16_t Y_MAX = ST7735_TFTWIDTH_128;

    const int16_t BORDER_X_MIN = X_MIN;
    const int16_t BORDER_X_MAX = X_MAX;
    const int16_t BORDER_Y_MIN = Y_MIN + 20;
    const int16_t BORDER_Y_MAX = Y_MAX;

    const uint32_t UPDATE_DELAY_MS = 30;

    const float DEFAULT_BALL_VX = 1.5;
    const float DEFAULT_BALL_VY = 1.7;

    enum class BreakoutEvent
    {
        BRICK_COLLIDE = 1,
        WIN_CONDITION = 2,
        BALL_OUT_OF_BOUNDS = 3,
        PADDLE_COLLIDE = 4
    };

    void send_event(BreakoutEvent event) {
        dodobot_serial::info->write("breakout", "ud", CURRENT_TIME, (int)(event));
    }

    int16_t sign(int16_t x) {
        return (x > 0) - (x < 0);
    }

    class GameObject {
    protected:
        int16_t hitbox_x, hitbox_y;
        int16_t hitbox_w, hitbox_h;
        int16_t hitbox_right, hitbox_bottom;
        int16_t hitbox_cx, hitbox_cy;


    public:
        bool is_hidden;

        GameObject(): hitbox_x(0), hitbox_y(0), hitbox_w(0), hitbox_h(0)
        {
            is_hidden = false;
            update_hitbox_vars();
        }
        GameObject(int16_t x, int16_t y, int16_t w, int16_t h):
            hitbox_x(x), hitbox_y(y), hitbox_w(w), hitbox_h(h)
        {
            is_hidden = false;
            update_hitbox_vars();
        }

        void update_hitbox_vars()
        {
            hitbox_right = hitbox_x + hitbox_w;
            hitbox_bottom = hitbox_y + hitbox_h;
            hitbox_cx = (hitbox_x + hitbox_right) / 2;
            hitbox_cy = (hitbox_y + hitbox_bottom) / 2;
        }

        bool has_collided(GameObject* other)
        {
            if (is_hidden || other->is_hidden) {
                return false;
            }
            if (hitbox_x >= other->hitbox_right || other->hitbox_x >= hitbox_right) {
                return false;
            }
            if (hitbox_y >= other->hitbox_bottom || other->hitbox_y >= hitbox_bottom) {
                return false;
            }
            return true;
        }

        char collision_dir(GameObject* other)
        {
            char dir = 'x';
            int8_t dirs = 0;
            if (hitbox_right > other->hitbox_right || hitbox_x < other->hitbox_x) {
                dir = 'x';
                dirs++;
            }
            if (hitbox_bottom > other->hitbox_bottom || hitbox_y < other->hitbox_y) {
                dir = 'y';
                dirs++;
            }
            if (dirs == 2) {
                return 'b';
            }
            else {
                return dir;
            }
        }

        void hide() {
            is_hidden = true;
        }

        void show() {
            is_hidden = false;
        }

        int update() { return 0; }
        void draw() {  }
    };

    class Ball: public GameObject {
    private:
        float x, y;
        float prev_x, prev_y;
        float x0, y0;
        uint16_t radius;
        uint16_t color;
        float bounce_factor;

        int16_t x_border_max, x_border_min;
        int16_t y_border_max, y_border_min;
    public:
        float vx, vy;

        Ball(uint16_t r, uint16_t c, float bounce): radius(r), color(c), bounce_factor(bounce)
        {
            reset(0, 0);
            prev_x = x0;
            prev_y = y0;

            x_border_max = BORDER_X_MAX;
            x_border_min = BORDER_X_MIN;
            y_border_max = BORDER_Y_MAX;
            y_border_min = BORDER_Y_MIN;
        }

        int16_t get_x()  { return (int16_t)x; }
        int16_t get_y()  { return (int16_t)y; }
        uint16_t get_radius()  { return radius; }

        void reset(float x0, float y0)
        {
            vx = 0.0;
            vy = 0.0;
            x = x0;
            y = y0;
            update_hitbox();
        }

        void bounce(char dir)
        {
            switch (dir) {
                case 'x': vx = -vx; break;
                case 'y': vy = -vy; break;
                case 'b': vx = -vx; vy = -vy; break;
            }
            vx += rand_bounce();  // add a small random factor to the bounce
            vy += rand_bounce();
        }

        float rand_bounce()
        {
            randomSeed(micros());
            long rand_num = random(0, 1000000);
            return bounce_factor * (float)rand_num / 1000000;
        }

        void update_hitbox()
        {
            hitbox_x = x - radius;
            hitbox_y = y - radius;
            hitbox_w = 2 * radius;
            hitbox_h = 2 * radius;
            update_hitbox_vars();
        }

        int update()
        {
            float new_x = x + vx;
            float new_y = y + vy;

            if (new_x > x_border_max - radius || new_x < x_border_min + radius) {
                bounce('x');
                new_x = x + vx;
            }

            if (new_y < y_border_min + radius) {
                bounce('y');
                new_y = y + vy;
            }

            prev_x = x;
            prev_y = y;

            if (new_y > y_border_max - radius) {
                return 1;
            }
            else {

                x = new_x;
                y = new_y;
            }

            update_hitbox();
            return 0;
        }

        void draw() {
            dodobot_display::tft.fillCircle((int16_t)prev_x, (int16_t)prev_y, radius, ST77XX_BLACK);
            dodobot_display::tft.fillCircle((int16_t)x, (int16_t)y, radius, color);
            dodobot_display::tft.drawCircle((int16_t)x, (int16_t)y, radius, ST77XX_WHITE);
        }
    };

    class Brick: public GameObject {
    private:
        uint16_t color;
        bool has_been_drawn;
        int16_t border;

    public:
        Brick (int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c) : GameObject(x, y, w, h) {
            color = c;
            has_been_drawn = false;
            border = 1;
        }

        // void update() {
        //
        // }

        void set_color(uint16_t c) {
            color = c;
            has_been_drawn = false;
        }

        void draw() {
            if (is_hidden) {
                return;
            }
            // if (has_been_drawn) {
            //     return;
            // }
            dodobot_display::tft.fillRect(hitbox_x + border, hitbox_y + border, hitbox_w - (2 * border), hitbox_h - (2 * border), color);
            has_been_drawn = true;
        }

        void hide() {
            dodobot_display::tft.fillRect(hitbox_x, hitbox_y, hitbox_w, hitbox_h, ST77XX_BLACK);
            is_hidden = true;
        }
    };

    class BrickCollection {
    private:
        int16_t start_x, start_y;
        int16_t brick_width, brick_height;
        uint16_t brick_color;

        String level;
        Brick** bricks;
        size_t bricks_len;
        bool loaded_flag;

        void create_brick(size_t index, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c) {
            if (bricks == NULL || index >= bricks_len) {
                return;
            }
            bricks[index] = new Brick(x, y, w, h, c);
        }

        size_t num_bricks()
        {
            size_t count = 0;
            for (size_t index = 0; index < level.length(); index++) {
                switch (level.charAt(index)) {
                    case 'w':
                    case '#':
                    case 'b':
                    case 'g':
                    case 'r':
                    case 'c':
                    case 'm':
                    case 'y':
                    case 'k':
                    case 'a':
                        count++;
                }
            }
            return count;
        }

    public:
        BrickCollection(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c):
                start_x(x), start_y(y), brick_width(w), brick_height(h), brick_color(c) {
            bricks_len = 0;
            level = "";
            loaded_flag = false;
        }

        bool is_loaded() {
            return loaded_flag;
        }

        bool create_level(String l)
        {
            if (loaded_flag) {
                dodobot_serial::println_info("A level is already loaded.");
                return false;
            }
            loaded_flag = true;

            level = l;
            bricks_len = num_bricks();
            if (bricks_len == 0) {
                return false;
            }

            bricks = new Brick*[bricks_len];
            size_t brick_index = 0;

            int16_t x = start_x;
            int16_t y = start_y;
            uint16_t brick_color = ST77XX_WHITE;
            bool should_create_brick = false;
            for (size_t index = 0; index < level.length(); index++) {
                switch (level.charAt(index)) {
                    case '\n':   // row return
                        x = start_x;
                        y += brick_height;
                        break;

                      // brick here
                      case 'w':  // white
                      case '#':  brick_color = ST77XX_WHITE; should_create_brick = true; break;  // white
                      case 'b':  brick_color = ST77XX_BLUE; should_create_brick = true; break;  // blue
                      case 'g':  brick_color = ST77XX_GREEN; should_create_brick = true; break;  // green
                      case 'r':  brick_color = ST77XX_RED; should_create_brick = true; break;  // red
                      case 'c':  brick_color = ST77XX_CYAN; should_create_brick = true; break;  // cyan
                      case 'm':  brick_color = ST77XX_MAGENTA; should_create_brick = true; break;  // magenta
                      case 'y':  brick_color = ST77XX_YELLOW; should_create_brick = true; break;  // yellow
                      case 'k':  brick_color = ST77XX_GRAY; should_create_brick = true; break;  // gray
                      case 'a':  brick_color = ST77XX_ORANGE; should_create_brick = true; break;  // orange

                    default:
                        x += brick_width;
                        break;  // any other character = no brick
                }
                if (should_create_brick)
                {
                    create_brick(brick_index, x, y, brick_width, brick_height, brick_color);
                    x += brick_width;
                    brick_index++;
                }
                should_create_brick = false;
            }
            return true;
        }

        void delete_level()
        {
            if (!loaded_flag) {
                dodobot_serial::println_info("Level is already deleted.");
                return;
            }
            loaded_flag = false;

            for (size_t index = 0; index < bricks_len; index++) {
                if (bricks[index] == NULL) {
                    continue;
                }
                bricks[index]->hide();
                delete bricks[index];
            }
            delete[] bricks;
            bricks = NULL;
            dodobot_serial::println_info("Level deleted.");
        }

        void reset_level()
        {
            for (size_t index = 0; index < bricks_len; index++) {
                if (bricks[index] == NULL) {
                    continue;
                }
                bricks[index]->show();
            }
        }

        void draw()
        {
            for (size_t index = 0; index < bricks_len; index++) {
                if (bricks[index] == NULL) {
                    continue;
                }
                bricks[index]->draw();
            }
        }

        bool has_ball_collided(Ball* ball) {
            for (size_t index = 0; index < bricks_len; index++) {
                if (ball->has_collided(bricks[index]))  {
                    ball->bounce(ball->collision_dir(bricks[index]));
                    bricks[index]->hide();
                    return true;
                }
            }
            return false;
        }

        size_t num_shown_bricks() {
            size_t count = 0;
            for (size_t index = 0; index < bricks_len; index++) {
                if (!bricks[index]->is_hidden) {
                    count++;
                }
            }
            return count;
        }

        size_t get_num_bricks()  { return bricks_len; }

        ~BrickCollection() {
            delete_level();
        }
    };

    class Paddle: public GameObject {
    private:
        float x, y;
        float prev_x, prev_y;
        float x0, y0;
        int16_t w, h;
        float vx, vy;
        uint16_t color;
        uint32_t move_timer;

    public:
        Paddle(float x0, float y0, int16_t w, int16_t h, uint16_t c): x0(x0), y0(y0), w(w), h(h), color(c)
        {
            reset();
        }

        int16_t get_ball_x(uint16_t r)  { return x + w / 2; }
        int16_t get_ball_y(uint16_t r)  { return y - r; }

        void reset()
        {
            move_timer = CURRENT_TIME;
            vx = 0;
            vy = 0;
            x = x0;
            y = y0;
            prev_x = x0;
            prev_y = y0;
            update_hitbox();
        }

        void update_hitbox()
        {
            hitbox_x = x;
            hitbox_y = y;
            hitbox_w = w;
            hitbox_h = h;
            update_hitbox_vars();
        }

        void set_vx(float new_vx) {
            move_timer = CURRENT_TIME;
            vx = new_vx;
        }

        float get_vx()  { return vx;}

        void reset_timer() {
            move_timer = CURRENT_TIME;
        }

        int update()
        {
            if (CURRENT_TIME - move_timer > 150) {
                vx = 0.0;
            }
            float new_x = x + vx;

            if (new_x < BORDER_X_MIN) {
                new_x = BORDER_X_MIN;
            }
            if (new_x + w > BORDER_X_MAX) {
                new_x = BORDER_X_MAX - w;
            }

            prev_x = x;
            x = new_x;
            update_hitbox();
            return 0;
        }

        void set_ball_speed(Ball* ball) {
            int16_t impact_offset = ball->get_x() - ((int16_t)x + w / 2);
            float paddle_strike_percent = fabs((float)impact_offset * 2 / (float)w);
            if (paddle_strike_percent >= 0.9) {
                ball->vx = DEFAULT_BALL_VX * 2.0;
            }
            else if (paddle_strike_percent >= 0.7) {
                ball->vx = DEFAULT_BALL_VX * 1.5;
            }
            else if (paddle_strike_percent >= 0.2) {
                ball->vx = DEFAULT_BALL_VX;
            }
            else {
                ball->vx = DEFAULT_BALL_VX * 0.8;
            }
            if (impact_offset > 0) {
                if (ball->vx < 0) {
                    ball->vx = -ball->vx;
                }
            }
            else {
                if (ball->vx > 0) {
                    ball->vx = -ball->vx;
                }
            }
            ball->vy = -ball->vy;
        }

        void draw() {
            dodobot_display::tft.fillRect(prev_x, prev_y, w, h, ST77XX_BLACK);
            dodobot_display::tft.fillRect((int16_t)x, (int16_t)y, w, h, color);
            dodobot_display::tft.drawRect((int16_t)x, (int16_t)y, w, h, ST77XX_WHITE);
        }
    };


    BrickCollection* bricks = new BrickCollection(BORDER_X_MIN, BORDER_Y_MIN, 20, 10, ST77XX_WHITE);
    Paddle* paddle = new Paddle(80, Y_MAX - 5, 40, 5, ST77XX_DARKER_BLUE);
    Ball* ball = new Ball(5, ST77XX_DARKER_BLUE, 0.25);

    bool all_destroyed = false;
    size_t num_strikeouts = 0;
    String victory_message = "You did it! Press Enter.";
    uint32_t level_start_time = 0;
    String level_name = "BREAKOUT";
    String default_level_config = "ooo##ooo\noo####oo\nooo##ooo\noooooooo";
    String level_config = default_level_config;

    void reset_ball_to_paddle() {
        ball->reset(paddle->get_ball_x(ball->get_radius()), paddle->get_ball_y(ball->get_radius()));
    }

    void update_scoreboard()
    {
        dodobot_display::tft.setCursor(0, 0);
        dodobot_display::tft.print("bricks: ");
        dodobot_display::tft.print(bricks->num_shown_bricks());
        dodobot_display::tft.print(" of ");
        dodobot_display::tft.print(bricks->get_num_bricks());
        dodobot_display::tft.println("  ");
        dodobot_display::tft.print("time: ");
        if (level_start_time == 0) {
            dodobot_display::tft.print("0");
        }
        else {
            float time = (float)(CURRENT_TIME - level_start_time) / 1000.0;
            dodobot_display::tft.print(time);
        }
        dodobot_display::tft.print("      ");

        dodobot_display::tft.setCursor(80, dodobot_display::tft.getCursorY());
        dodobot_display::tft.print("outs: ");
        dodobot_display::tft.print(num_strikeouts);

        uint16_t w, h;
        dodobot_display::textBounds(level_name, w, h);
        dodobot_display::tft.setCursor(dodobot_display::tft.width() - w, 0);
        dodobot_display::tft.print(level_name);

        dodobot_display::tft.drawFastHLine(BORDER_X_MIN, BORDER_Y_MIN - 1, BORDER_X_MAX - BORDER_X_MIN, ST77XX_WHITE);
    }

    void win_callback()
    {
        send_event(BreakoutEvent::WIN_CONDITION);
        update_scoreboard();
        ball->draw();

        int16_t  x1, y1;
        uint16_t w, h;
        dodobot_display::tft.getTextBounds(victory_message, 0, 0, &x1, &y1, &w, &h);

        dodobot_display::tft.setCursor((X_MAX - w) / 2, (Y_MAX - h) / 2);
        dodobot_display::tft.print(victory_message);
    }

    void recreate_level()
    {
        tft.fillScreen(ST77XX_BLACK);
        level_start_time = 0;
        num_strikeouts = 0;
        all_destroyed = false;
        reset_ball_to_paddle();
        bricks->delete_level();
        if (!bricks->create_level(level_config)) {
            level_config = default_level_config;
            bricks->create_level(level_config);
        }
        dodobot_serial::println_info("Available memory: %d", dodobot::free_mem());
    }

    void enter_event()
    {
        recreate_level();
        // bricks->reset_level();
    }
    void right_event() {
        paddle->set_vx(2.0);
    }
    void left_event() {
        if (ball->vx == 0.0 && ball->vy == 0.0) {
            if (level_start_time == 0) {
                level_start_time = CURRENT_TIME;
            }
            ball->vx = -DEFAULT_BALL_VX;
            ball->vy = -DEFAULT_BALL_VY;
        }
        paddle->set_vx(-2.0);
    }
    void repeat_key_event() {
        paddle->reset_timer();
    }

    void draw()
    {
        if (all_destroyed) {
            return;
        }
        update_scoreboard();

        int result = ball->update();
        if (result == 1) {
            send_event(BreakoutEvent::BALL_OUT_OF_BOUNDS);
            reset_ball_to_paddle();
            num_strikeouts++;
        }
        ball->draw();

        paddle->update();
        paddle->draw();

        if (ball->vx == 0.0 && ball->vy == 0.0)
        {
            if (paddle->get_vx() != 0.0) {
                if (level_start_time == 0) {
                    level_start_time = CURRENT_TIME;
                }
                if (paddle->get_vx() > 0.0) {
                    ball->vx = DEFAULT_BALL_VX;
                    ball->vy = -DEFAULT_BALL_VY;
                }
                else {
                    ball->vx = -DEFAULT_BALL_VX;
                    ball->vy = -DEFAULT_BALL_VY;
                }
            }
        }

        bricks->draw();
        if (bricks->has_ball_collided(ball)) {
            send_event(BreakoutEvent::BRICK_COLLIDE);
        }

        all_destroyed = bricks->num_shown_bricks() == 0;
        if (all_destroyed) {
            win_callback();
        }

        if (ball->has_collided(paddle))  {
            // ball->bounce(ball->collision_dir(paddle));
            send_event(BreakoutEvent::PADDLE_COLLIDE);
            paddle->set_ball_speed(ball);
        }
    }

    void on_load()
    {
        recreate_level();
    }

    void on_unload()
    {
        bricks->delete_level();
    }
}

#endif  // __DODOBOT_BREAKOUT_H__
