<h1 align="center">Autonomous vehicle platooning architecture in Carla</h1>
This proejct was a part of a dissertation to explore solving how to achieve coordination of autonomous vehicles 
(AVs) through using available literature and ideas. With the many types of ways to 
coordinate vehicles, the project will primarily focus on AVs platooning. Platooning involves 
having vehicles travel in groups with small distances between themselves (usually less 
than a metre). The outcome of the project was an architecture that has been designed 
and programmed for coordinating AVs simulated in the CARLA simulator. The purpose of 
exploring this topic is to determine whether the coordination architecture reduces traffic 
congestion by a significant amount by increasing the capacity on the roads. Additionally, 
the architecture has been implemented with multi-lane merging to investigate the stability 
of including advanced platooning manoeuvres. 

<h2 align="center"><b>Examples of manoeuvres</b></h2>
<p align="center"><b>Leave manoeuvre</b></p>

In step 1, the platoon is reaching 
the point where the middle member will leave. In step 2, the member is at the leaving 
point, so the captain removes the member from the platoon. In step 3, the member that 
left cannot find a new platoon to join so it creates its own, and the gap the member left in 
the previous platoon starts to close.
<p align="center">
  <img align="center" src="https://github.com/OliverShv/Coordinating-Autonomous-Vehicles-to-Reduce-Traffic-Congestion/assets/20969412/63118762-4956-45de-8bd9-6c6eefba537c">
</p>

<p align="center"><b>Multi-lane merge manouvre</b></p>

In step 1, two platoons are spawned next 
to each other with the platoon in the middle lane wanting to merge with the platoon on the 
inner lane. In step 2, the vehicles in both lanes create gaps. In step 3, the platoon in the 
middle lane uses the gaps to insert its vehicles in the inner lane. In step 4, the vehicles 
have merged into one platoon and are instructed to drive closely to each other.
<p align="center">
  <img align="center" src="https://github.com/OliverShv/Coordinating-Autonomous-Vehicles-to-Reduce-Traffic-Congestion/assets/20969412/795fd8c3-1b4b-433a-af7d-d5b499ded406">
</p>
<h2 align="center"><b>Results</b></h2>
Figures 4.1 and 4.2 are the heatmaps of the road network to show the effect of the volume
to-capacity for the entire simulation for both platooning and autonomous vehicles roaming 
respectively. The purpose of the heatmaps is to provide a visual aid of what areas of the 
road network were affected the most. Three colours indicate the level of volume-to
capacity a lane has experienced throughout every simulation compared against the 
maximum volume-to-capacity recorded. Green indicates that the lane experienced 30% 
or under, yellow indicates that the lane has experienced between 30% to 70%, and red 
indicates that the lane has experienced 70% or over.  
Comparing single-lane platooning in figure 4.1 to the benchmark in figure 4.2, there is a 
clear difference observed. Single-lane platooning only achieved at least 70% of the 
maximum volume-to-capacity once where the benchmark achieved that level on 7 
49 
 
different lanes. Both single-lane platooning, and the benchmark has a similar number of 
lanes between 30% to 70% of the maximum volume-to-capacity, but single-lane 
platooning had a higher volume of lanes under 30% of the maximum volume-to-capacity.  
Moving on to Figure 4.3, when comparing the average volume-to-capacity per second of 
the benchmark and platoon, there is a clear difference between the results. Viewing table 
4.2, on average each second, with platooning enabled, there was a 15% increase in road 
capacity when compared to the benchmark. Additionally, platooning also had a lower 
maximum and minimum road capacity when compared to the benchmark.

<p align="center">
  <table>
    <tr>
      <td> <img align="center" src="https://github.com/OliverShv/Coordinating-Autonomous-Vehicles-to-Reduce-Traffic-Congestion/assets/20969412/92b818af-8f5f-44f0-8518-0d2d830aad89"></td>
      <td> <img align="center" src="https://github.com/OliverShv/Coordinating-Autonomous-Vehicles-to-Reduce-Traffic-Congestion/assets/20969412/4ee2e753-218a-4d9d-935c-e48965cd0019"></td>
    </tr>
  </table>
</p>
