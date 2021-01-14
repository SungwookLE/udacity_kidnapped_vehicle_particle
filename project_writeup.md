# PROJECT WRITEUP

> **AUTHOR**: SungwookLE<br/> > **DATE**: '21.1/14<br/> > **NOTE**: This document is written for explaining the pipeline of the code<br/>

[//]: # "Image References"
[image1]: ./data/map_data.png "Model Visualization"
[image2]: ./data/Simulation_results.png "Pass"

<br/>
### Requirements and Data, Introduction

> > **Requirement**: Check Here! [Rubric](https://review.udacity.com/#!/rubrics/747/view).
> > **Given Map Data**: Below image. ![alt_text][image1].
> > **Introduction**: This project for localization of ego-vehicle in given map and sensor observation. Using Particle filter algorithm, this code would localize the ego vehicle position according to time stamp. Particle filter is one of the probabilitic approach filter, and we assume the X,Y plain field excluding height of the field. Map Data points and observation points are associated using the nearest neighbor algorithm that is one of the simplest data association.
> > <br/>

### MAIN

First, We are looking at main.cpp file.

```C++
 // Read map data
Map map;
if (!read_map_data("../data/map_data.txt", map)) {
   std::cout << "Error: Could not open map file" << std::endl;
   return -1;
}

// Create particle filter
ParticleFilter pf;

h.onMessage([&pf,&map,&delta_t,&sensor_range,&sigma_pos,&sigma_landmark]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode){}
```

Map class and ParticleFilter class is declared as map and pf.
Using h object that is class of the uWS::Hub, This code communicates with the simulation program(term2_udacity_simulator).
The object pf is the key in this project.

```C++
 if (!pf.initialized()) {
            // Sense noisy position data from the simulator
            double sense_x = std::stod(j[1]["sense_x"].get<string>());
            double sense_y = std::stod(j[1]["sense_y"].get<string>());
            double sense_theta = std::stod(j[1]["sense_theta"].get<string>());

            pf.init(sense_x, sense_y, sense_theta, sigma_pos);
```

Above Code is about initialization, If it is the first loop (before initialized), Using sense_x, y, theta( we assume it from GPS ). particle filter initialization is executed.

Let's look at the init member in particle_filter.cpp

> > > ParticleFilter::init()

```c++
  num_particles = 20;  // TODO: Set the number of particles
  std::random_device rd;
  std::mt19937 gen(rd());

  std::normal_distribution<double> dist_x(0, std[0]);
  std::normal_distribution<double> dist_y(0, std[1]);
  std::normal_distribution<double> dist_theta(0, std[2]);

  for (int i = 0; i < num_particles; ++i){
    Particle particle;
    particle.id = i + 1;
    particle.x = x+dist_x(gen);
    particle.y =  y+dist_y(gen);
    particle.theta = fmod(theta+dist_theta(gen), 2*M_PI);
    particle.weight = 1;
    particles.push_back(particle);

  }
```

Define the number of particles . and using random seed, we make the gaussian distribution in each sensor value (x,y,yaw).
In for loop, we initialized each particle with each data in standard deviation.

Back to the main.cpp.
After initialized, the code run the prediction() member.

```c++
    else // after initialized step
        {
            // Predict the vehicle's next state from previous
            //   (noiseless control) data.
            double previous_velocity = std::stod(j[1]["previous_velocity"].get<string>());
            double previous_yawrate = std::stod(j[1]["previous_yawrate"].get<string>());

            pf.prediction(delta_t, sigma_pos, previous_velocity, previous_yawrate);


        }
```

Look at the particle_filter.cpp again.

> > > ParticleFilter::prediction()

```c++
  std::random_device rd;
  std::mt19937 gen(rd());

  std::normal_distribution<double> dist_x(0, std_pos[0]);
  std::normal_distribution<double> dist_y(0, std_pos[1]);
  std::normal_distribution<double> dist_theta(0, std_pos[2]);

  if (fabs(yaw_rate)> 0.000001){
    for (int i = 0; i < num_particles ; ++i){
      particles.at(i).x = particles[i].x + velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta)) + dist_x(gen);
      particles.at(i).y = particles[i].y + velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t)) + dist_y(gen);
      particles.at(i).theta = fmod(particles[i].theta + yaw_rate * delta_t + dist_theta(gen), 2*M_PI);
    }
  }
  else{ // defense for divide zero!
    for (int i = 0; i < num_particles ; ++i){
      particles.at(i).theta = fmod(particles[i].theta+ dist_theta(gen), 2*M_PI);
      particles.at(i).x = particles.at(i).x + (velocity * delta_t * cos(particles[i].theta))+ dist_x(gen);
      particles.at(i).y = particles.at(i).y + (velocity * delta_t * sin(particles[i].theta))+ dist_y(gen);
      std::cout << "Too small yaw rate input! " << std::endl;
    }
  }
}
```

Using Simple model, The next step location of each particle is predicted.
The first paragraph of the if codition is the model of bicycle model (in case yaw rate is existed).

Back to the main.cpp.

```c++
// Update the weights and resample
pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map);
pf.resample();
```

updateWeights member is calculte the probability using multi-variable gaussian function.
In simple way, updateWeights(), is doing that match the observation(sensor) data and landmark data in map_coordinate. and associate the landmark and observation data, finally calculate the score how to match well using gaussian function. that's all.
Let's look at the particle_filter.cpp.

> > > ParticleFilter::updateWeights()

```c++
for (int n = 0; n < num_particles; ++n){
    std::vector<LandmarkObs> transformed_observations{};
    std::vector<LandmarkObs> candidated_landmarks{};

    // STEP1. transformed the observation data
    for (int i = 0; i < observations.size(); ++i){
      LandmarkObs transformed_observation;

      transformed_observation.x = cos(particles[n].theta) * observations[i].x - sin(particles[n].theta) * observations[i].y + particles[n].x;
      transformed_observation.y = sin(particles[n].theta) * observations[i].x + cos(particles[n].theta) * observations[i].y + particles[n].y;

      transformed_observations.push_back(transformed_observation);
    }

    // STEP2. data association(Nearest Neighbor)
    for (int j = 0; j < map_landmarks.landmark_list.size(); ++j){
      if( dist(particles[n].x, particles[n].y, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f ) <=sensor_range ){
      //if( (fabs(particles[n].x - map_landmarks.landmark_list[j].x_f ) <= sensor_range ) && ( fabs(particles[n].y - map_landmarks.landmark_list[j].y_f ) <= sensor_range ) ){
        LandmarkObs candidate_landmark;
        candidate_landmark.id = map_landmarks.landmark_list[j].id_i;
        candidate_landmark.x = map_landmarks.landmark_list[j].x_f;
        candidate_landmark.y = map_landmarks.landmark_list[j].y_f;
        candidated_landmarks.push_back(candidate_landmark);
      }
    }

    if (candidated_landmarks.size() > 0 ){
      dataAssociation(candidated_landmarks, transformed_observations);

      // STEP3. Update the weights of each particle using a mult-variate Gaussian distribution.
      particles[n].weight = 1; // re-initialization
      for (int k = 0; k < transformed_observations.size(); ++k){
        for ( int p =0; p < candidated_landmarks.size() ; ++p){
          if ( candidated_landmarks[p].id ==  transformed_observations[k].id){
            particles[n].weight *= multiv_prob(std_landmark[0], std_landmark[1], transformed_observations[k].x, transformed_observations[k].y, candidated_landmarks[p].x, candidated_landmarks[p].y);
          }
        }
      }
    }
    else{
      std::cout << "NO Detected Landmark in Sensor Range !!!!" << std::endl;
    }
}
```

Each particle are exectuted same process<br/>
**STEP1.** transformed the observation data respectively using rotation matrix.

$$
\left[ \begin{array}{c} \text{x}_m \\ \text{y}_m \\ 1 \end{array} \right] = \begin{bmatrix} \cos\theta & -\sin\theta & \text{x}_p \\ \sin\theta & \cos\theta & \text{y}_p \\ 0 & 0 & 1 \end{bmatrix} \times \left[ \begin{array}{c} \text{x}_c \\ \text{y}_c \\ 1 \end{array} \right]\\
xm​=xp​+(cosθ×xc​)−(sinθ×yc​)
\\ym​=yp​+(sinθ×xc​)+(cosθ×yc​)
$$

<br/>
**STEP2.** data association
Before data association , Check the landmark whether in sensor range or not.
For this, I defined `std::vector<LandmarkObs> candidated_landmarks{}`.
Next, `dataAssociation()` is called.
It is find the nearest data between landmark and observation data.

```C++
void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs>& observations) {
  float distance;

  for (int i = 0; i < observations.size(); ++i)
  {
    float prev_dist=10000.0;
    for (int j = 0; j < predicted.size(); ++j){
      distance = dist(predicted[j].x, predicted[j].y , observations[i].x, observations[i].y);
      //((predicted[j].x - observations[i].x) * (predicted[j].x - observations[i].x) + (predicted[j].y - observations[i].y) * (predicted[j].y - observations[i].y));
      if (distance< prev_dist){
        observations[i].id = predicted[j].id;
        prev_dist = distance;
      }
    }
  }
}
```

The matching information is saved in observations object as ID. This information well transfered out of this function. Because argument is declared as reference( call by reference)
<br/>
**STEP3.** updating weight to each particle
Update the weights of each particle using a mult-variate Gaussian distribution.

```c++
double ParticleFilter::multiv_prob(double sig_x, double sig_y, double x_obs, double y_obs,
                   double mu_x, double mu_y) {
  // calculate normalization term
  double gauss_norm;
  gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

  // calculate exponent
  double exponent;
  exponent = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2)))
               + (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));

  // calculate weight using normalization terms and exponent
  double weight;
  weight = gauss_norm * exp(-exponent);

  return weight;
}
```

This function returns weight calcuted using gaussian multi-variable distribution.

Now, Let's look at the `pf.resample()`.

> > > ParticleFilter::resample()

It is quite important to choose the proper candidates among particles.
The idea is that using `std::discrete_distribution<> rand(weights_for_resample.begin(), weights_for_resample.end())`
proper particles is choosen according to the size of weight.
It other words, the bigger weight is , the bigger probability to choose is.

Also, there is another way how to implement like annotation part in the below code called resampling_wheel.
But, for more clearly implementation, using discrete_distribution() is adapted.

```c++
void ParticleFilter::resample() {
  std::random_device rd;
  std::mt19937 gen(rd());

  float sum = 0;
  std::vector<float> weights_for_resample;
  for (int j = 0; j < num_particles; ++j)
  {
    sum += particles[j].weight;
    weights_for_resample.push_back(particles[j].weight);
  }

  std::vector<Particle> samp_particles;
  // (1) Using resampling_wheel method !!!!!
  /*std::uniform_real_distribution<double> rand(0, 1);
  float beta = 0;
  int index = int(rand(gen) * num_particles);

  for (int i = 0; i < num_particles; ++i)
  {
    beta += rand(gen) * 2.0 * std::max_element(particles.begin(), particles.end(), [](Particle a, Particle b) { return (a.weight < b.weight); })->weight;

    while( particles[index].weight < beta ){
      beta = beta - particles[index].weight;
      index = (index + 1) % num_particles;
    }

    samp_particles.push_back( particles[index]);
  }*/

  // (2) Using discrete_distribution !!!!!
  std::discrete_distribution<> rand(weights_for_resample.begin(), weights_for_resample.end());
  for (int q = 0; q < num_particles; ++q){
    int index = rand(gen);
    samp_particles.push_back(particles[index]);
  }
  particles = samp_particles;
}
```

Those are the thing that I implemented about particle_filter class.
Let's back to the main.cpp.
And then, the particle weight and x,y location is transfered to simulation program.
<br/>

### Results

> > Simulation returns pass like below image ![alt_text][image2].
> > You can see the landmark as black 'o', and blue 'o' is the estimated vehilce location (x,y,yaw).
