/*
 * Title: MY BOID FLOCKING BEHAVIOR SIMULATION based on Conrad Parker's pseudocode algorithm.
 * Athor: Mckenzie J. Regalado
 * From: Naval, Biliran, Philippines
 * Date: 8/25/2023
 */

/*
 * Reference: https://vergenet.net/~conrad/boids/pseudocode.html
*/

#include <SFML/Graphics.hpp>
#include <SFML/Window/Mouse.hpp>

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <random>

#define PI 3.14159265

#define WINDOW_WIDTH  720
#define WINDOW_HEIGHT 360

std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> d_float(0.0, 1.0);
std::uniform_int_distribution<> d_int_color(1, 255);

std::uniform_int_distribution<> d_int_perchTimer(100, 200);

class Boid
{
private :
    sf::Vector2f points[3] {};

public :
    float r = 1.0;

    sf::Vector2f pos{};
    sf::Vector2f vel{};
    sf::ConvexShape entity{};

    bool isPerching = false;

    /* Determine how long the boid will perch for */
    int perchTimer{};

    unsigned int Id{};

    Boid(sf::Vector2f& init_pos, unsigned int id) : pos{ init_pos }, Id{ id }
    {
        // random perching time.
        perchTimer = d_int_perchTimer(gen);

        // Make a new 2D unit velocity vector from a random angle.
        vel = random2D();

        // Initialize vertices.
        points[0].y =  0.0;
        points[0].y = -r*2.0;
        points[1].x = -r;
        points[1].y =  r*2.0;
        points[2].x =  r;
        points[2].y =  r*2.0;

        // set boid's points.
        entity.setPointCount(3);
        entity.setPoint(0, points[0]);
        entity.setPoint(1, points[1]);
        entity.setPoint(2, points[2]);

        // set the boid with random color.
        sf::Color randomFillColor = randomRGBcolor();
        entity.setFillColor(randomFillColor);

        // set the boid's outline thickness and color.
        entity.setOutlineThickness(1);
        entity.setOutlineColor(sf::Color(255, 255, 255));

    }

    void setPerchingValue(bool v)
    {
        isPerching = v;
    }

    sf::Vector2f getPosition()
    {
        return pos;
    }
    sf::Vector2f getVelocity()
    {
        return vel;
    }

private :
    sf::Vector2f random2D()
    {
        float angle = d_float(gen) * (PI*2);
        float length = 1.0f;
        return sf::Vector2f(length * cosf(angle), length * sinf(angle));
    }

    sf::Color randomRGBcolor()
    {
        sf::Color color{};
        color.r = d_int_color(gen);
        color.g = d_int_color(gen);
        color.b = d_int_color(gen);
        return color;
    }
};


class Flock
{

private :
    std::vector<Boid> boids{};
    unsigned int n_boids;

    /* Tunable parameters. */
    float m               = 1.0f;
    float turnfactor      = 10.0f;

    float safety_distance = 30.0f, perceptual_distance = 150.0f;

    float as = 0.05f;

    float ac = 0.0005f;

    float aa = 0.05f;

    float velMax = 10.0f, velMin = 5.0f, clamping_mag = 0.01f, schooling_mag = 0.125;

    sf::Vector2f v0{}, v1{}, v2{}, v3{}, v4{};

    // Place position
    int placePosX = WINDOW_WIDTH/2;
    int placePosY = WINDOW_HEIGHT/2;

    int GroundLevel = WINDOW_HEIGHT-10;

public :
    Flock(unsigned int number_of_boids, sf::Vector2f init_pos) : n_boids{ number_of_boids }
    {
        for (unsigned int i=0; i<n_boids; i++)
        {
            boids.push_back(Boid(init_pos, i));
        }
    }
    void update()
    {
        for (unsigned int i=0; i<n_boids; i++)
        {
            Boid *b = &boids[i];
            /* before attempting to apply the rules we check if the boid is perching. */
            if (b->isPerching)
            {
                /* and if so we decrement the timer b.perchTimer and skip the rest of the loop.  */
                if (b->perchTimer > 0)
                {
                    b->perchTimer -= 1;
                    continue;
                }
                else
                {
                    b->setPerchingValue(false);
                }
            }

            v0 = tend_to_place(boids[i]);
            v1 = Fs(boids[i]);                                 // seperation force
            v2 = Fc(boids[i]) - boids[i].getPosition();        // cohesion force
            v3 = Fa(boids[i]) - boids[i].getVelocity();        // allignment force
            v4 = bound_position(boids[i]);

            v0.x *= m;
            v0.y *= m;

            v1.x *= as;
            v1.y *= as;

            v2.x *= ac;
            v2.y *= ac;

            v3.x *= aa;
            v3.y *= aa;

            boids[i].vel += v0;
            boids[i].vel += v1;
            boids[i].vel += v2;
            boids[i].vel += v3;
            boids[i].vel += v4;

            limit_velocity(boids[i]);
            boids[i].pos += boids[i].getVelocity();
        }
    }

    void render(sf::RenderWindow& window)
    {
        for (unsigned int i=0; i<n_boids; i++)
        {
            Boid *boid = &boids[i];
            float angleRotation = heading(boid->getVelocity()) + 90.0;
            boid->entity.setPosition(boid->getPosition());
            boid->entity.setRotation(angleRotation);
            window.draw(boid->entity);
        }
    }

    void setPlace(int posX, int posY)
    {
        placePosX = posX;
        placePosY = posY;
    }
private :
    float heading(sf::Vector2f v)
    {
        return atan2(v.y, v.x) * 180 / PI;
    }

    float squaredMagf(sf::Vector2f v)
    {
        return (float) (v.x*v.x) + (v.y*v.y);
    }

    void normalized(sf::Vector2f &v)
    {
        sf::Vector2f normalized;

        float denom = magf(v);
        v.x /= denom;
        v.y /= denom;
    }

    float magf(sf::Vector2f v)
    {
        return sqrtf(squaredMagf(v));
    }

    float distance(sf::Vector2f p1, sf::Vector2f p2)
    {
        return magf(p1 - p2);
    }

    sf::Vector2f Fc(Boid &bj)
    {
        sf::Vector2f pcj{0.0f, 0.0f};
        unsigned int neighbors = 0;

        for (unsigned int i=0; i<n_boids; i++)
        {
            Boid *otherBoid = &boids[i];

            // If the boid is not equal to itself.
            if (otherBoid->Id != bj.Id)
            {
                float dist = distance(bj.getPosition(), otherBoid->getPosition());
                if ((dist > 0.0f) && (dist < perceptual_distance))
                {
                    pcj += otherBoid->getPosition();
                    neighbors++;
                }
            }
        }

        if (neighbors > 0) {
            pcj.x /= (float) neighbors;
            pcj.y /= (float) neighbors;

            pcj.x *= clamping_mag;
            pcj.y *= clamping_mag;
        } else {
            pcj.x = 0.0f;
            pcj.y = 0.0f;
        }
        return pcj;
    }

    sf::Vector2f Fs(Boid &bj)
    {
        sf::Vector2f c{0.0f, 0.0f};
        unsigned int neigbors = 0;

        for (unsigned int i=0; i<n_boids; i++)
        {
            Boid *otherBoid = &boids[i];
            // If the boid is not equal to itself.
            if (otherBoid->Id != bj.Id)
            {
                float dist = distance(bj.getPosition(), otherBoid->getPosition());
                if ((dist > 0.0f) && (dist < safety_distance))
                {
                    c += (bj.getPosition() - otherBoid->getPosition());
                    neigbors++;
                }
            }
        }

        if (neigbors > 0)
        {
            c.x /= neigbors;
            c.y /= neigbors;
        } else {
            c.x = 0.0f;
            c.y = 0.0f;
        }
        return c;
    }

    sf::Vector2f Fa(Boid &bj)
    {
        sf::Vector2f pvj{0.0f, 0.0f};
        unsigned int neighbors = 0;

        for (unsigned int i=0; i<n_boids; i++)
        {
            Boid *otherBoid = &boids[i];

            // If the boid is not equal to itself.
            if (otherBoid->Id != bj.Id)
            {
                float dist = distance(bj.getPosition(), otherBoid->getPosition());
                if ((dist > 0.0f) && (dist < perceptual_distance))
                {
                    pvj += otherBoid->getVelocity();
                    neighbors++;
                }
            }
        }

        if (neighbors > 0) {
            pvj.x /= (float) neighbors;
            pvj.y /= (float) neighbors;

            pvj.x *= schooling_mag;
            pvj.y *= schooling_mag;
        }
        else {
            pvj.x = 0.0f;
            pvj.y = 0.0f;
        }
        return pvj;
    }

    // Tend_to_place
    sf::Vector2f tend_to_place(Boid &b)
    {
        sf::Vector2f place(placePosX, placePosY);

        place -= b.getPosition();
        place.x /= 100.0;
        place.y /= 100.0;

        return place;
    }

    // Bound the position
    sf::Vector2f bound_position(Boid &b)
    {
        sf::Vector2f v{};

        // Perching
        if (b.getPosition().y > GroundLevel)
        {
            b.pos.y = GroundLevel;
            b.setPerchingValue(true);
        }

        if (b.getPosition().x < b.r)
        {
            v.x = turnfactor;
        }
        else if (b.getPosition().x > WINDOW_WIDTH - b.r)
        {
            v.x = -turnfactor;
        }

        if (b.getPosition().y < b.r)
        {
            v.y = turnfactor;
        }
        else if (b.getPosition().y > WINDOW_HEIGHT - b.r)
        {
            v.y = -turnfactor;
        }
        return v;
    }

    // Limit the velocity
    void limit_velocity(Boid &b)
    {
        float length = magf(b.getVelocity());
        if (length > 0.0f)
        {
            if (length > velMax)
            {
                normalized(b.vel);
                b.vel.x *= velMax;
                b.vel.y *= velMax;
            }
            if (length < velMin)
            {
                normalized(b.vel);
                b.vel.x *= velMin;
                b.vel.y *= velMin;
            }
        }
    }
};

int main(void)
{
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;

    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Flocking Simulation", sf::Style::Default, settings);

    /* Activating vertical synchronization will limit the number of frames displayed to the refresh rate of the monitor. */
    window.setVerticalSyncEnabled(true);

    // create flock
    Flock flock(50, sf::Vector2f(WINDOW_WIDTH/2, WINDOW_HEIGHT/2));

    while (window.isOpen())
    {

        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();

            if (event.type == sf::Event::MouseButtonPressed)
            {
                if (event.mouseButton.button == sf::Mouse::Left)
                {
                    flock.setPlace(event.mouseButton.x, event.mouseButton.y);
                }
            }
        }

        window.clear(sf::Color::Black);
        // update boids' position and velocity
        flock.update();
        // render boids
        flock.render(window);
        window.display();
    }

    return 0;
}