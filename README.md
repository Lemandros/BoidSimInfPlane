
# Boid Sim - Infinite plane
This program, built using the Qt UI framework, simulates the 2D [Vicsek model](https://en.wikipedia.org/wiki/Vicsek_model) on an infinite plane using an additional inward force on the convex hull. To run it, you will need [Qt](https://www.qt.io/). After opening the .pro file in the Qt editor, simply compile, play with the options and run the model!
Features of this program include:

 - Configurable density, speed and noise level parameters
 - Configurable noise type (scalar vs vectorial)
 - Various graphical aids such as choosing what to render and how often, tracking individual boids over many timesteps
 - Implementing leadership
 - Generation of various observables
 - Plotting of said observables
 - FFT of observables
 - Saving of observables, system configurations
 - Programming multiple runs of the models using various parameters for untouched simulation

## The Vicsek model
This program simulates the collective 2D motion of agents using the Vicsek model for the purposes of analysing its qualitative and quantitative behaviour.  It is composed of $N$ identical agents - Boids - which are defined by a position $\vec{r_i}$ and velocity $\vec{v_i}$, the length of which is a constant $v_0$ for all Boids $i$. After being initially placed using a configured density $\rho$ and an orientation $\theta_i=\arg\vec{v_i}$, chosen randomly, the simulation starts.
 At each timestep, all Boids synchronously inspect their surroundings in a radius $\ell_0$ for neighbouring Boids. 
They then update their orientation by averaging their own with any Boid found within that radius, adding a noise term, and move into that direction:
 $\theta_i\left(t+\Delta t)\right)=\arg\left(\sum_{\left[i,j\right]}\vec{v_j}\right)+2\pi\eta$ and 
 
$\vec{r_i}\left(t+\Delta t\right)=\vec{r_i}\left(t\right)+v_0\left(\cos\theta\left(t+\Delta t\right), \sin\theta\left(t+\Delta t\right)\right)$  with $\left[i,j\right]$ indicating the sum over those Boids $j$ which are within the radius $\ell_0$ of Boid $i$, including $i$ itself. The noise $\eta$ then runs between $0$ and $1$. The model seems very similar to  an off-lattice Ising system, or more precisely XY model, with the addition of movement, but behaves quite differently.
 
In the original model, an observable $P=\frac{1}{N}\left|\sum_i\vec{v_i}\right|$, the normalised absolute value of the sum of all velocity vectors, was used to distinguish between a state of collective motion ($P$ approaches $1$), and a state of disorder ($P$ approaches $0$). A phase change occurs around $\eta_c =0.65$, from an ordered state for $\eta<\eta_c$ and a disordered state above it.  The nature of the phase change, whether of first or second order, took some time to uncover, as the original model could only be probed properly using very large ($N>10^5$) systems.
### Vectorial noise
 Later on, another mechanism for adding noise was implemented, which was able to probe this behaviour for much smaller systems. This noise, to contrast it with the scalar form of the original model, was called vectorial noise, and it was added to the neighbour average as a random unit vector, multiplied with the number of neighbours:
 
 $\vec{v_i}\left(t+\Delta t\right)=v_0\mathcal{N}\left[\sum_{\left[i,j\right]}\vec{v_j}+n_B\eta\hat{n}\left(\xi\right)\right]$ with $n_B$ the number of neighbours, $\hat{n}\left(\xi\right)$ a random unit vector and $\mathcal{N}\left[\vec{u}\right]=\frac{\vec{u}}{\left|\vec{u}\right|}$ a normalisation operator.

The fact that collective motion can occur at all, was a surprising result. A seeming violation of the Mermin-Wagner theorem, the explanation eventually provided by Toner and Tu was that the system is not in equilibrium from the start, and that the constant speed provides an asymmetry in the propagation of errors with respect to the motion of the flock. Toner has been giving a great talk on the subject for a while now, titled "Fish gotta swim, birds gotta fly, I gotta do Feynman graphs til I die!" which I can recommend if a deeper analysis involving hydrodynamics is something you fancy. There are recordings out there.
## The infinite plane
The novel alteration this version of the model is to move the Boids onto an infinite plane. While the original model used periodic boundary conditions, this variant provides an infinite surface of movement. Due to noise, the unmodified model would eventually turn to a gaseous, noninteracting state. To provide cohesion, a subset of Boids, those forming the convex hull, are provided an inward nudge.
The exact form of this inward nudge can be chosen from 4 options ("force recipes"):
 

 - Local curvature analogous to surface tension
 - Near-neighbour to simulate the tendency to stay near neighbours on the hull
 - Far-neighbour to simulate the tendency to keep hull density uniform locally
 - Flock mean to simulate the tendency to follow the entire flock
Each force recipe gives unique results in terms of the shape of the hull, so I encourage you to play around with it.

# Disclaimer
I created this program for my MSc Theoretical Physics [thesis](https://studenttheses.universiteitleiden.nl/handle/1887/49517) in 2016-2017 and only recently decided to add this readme to the repo. It is no longer actively being worked on, but I have played around with it recently (September 2022) using up-to-date Qt and C++ on Windows. A modified version of this program, returning it to a periodic-boundary model, is still in use by my supervisor for a course in Statistical Physics. I may publish it in a separate repo.
From what I recall, there are some differences between compiling for Linux and Windows, specifically for enabling OpenMP to parallelize some functionality. In the past, I used both OS' to run the program, but your mileage may vary at this stage. It *should* only be a matter of editing the .pro file, but no guarantees. If not, try removing the OpenMP pragmas surrounding the interaction and movement loops in BoidSim2D.cpp. I still check Github every now and then so if anything fails, try and reach out :) 
# License
You're free to edit and reuse as you see fit, though I would love a mention if it ever turns into something worthwhile, like a publication!
