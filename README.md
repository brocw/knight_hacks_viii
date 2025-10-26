# Drone Route Optimizer

## Introduction
Submission repository for KnightHacks VIII and part of the NextEra Energy challenge track. Our team was drawn into this challenge because of NextEra's values as well as providing an opportunity to learn more about mathematics, algorithms, optimization, and other tools that we felt weren't provided by the other sponsor tracks. Since half of our team members had machine learning and algorithm experience, this track proved to be a challenging but interesting project. Because our team memembers wanted to push our understanding of algortihm and the interconnections between nodes and edges, NextEra gave us the opportunity to explore more mathematics at a mostly software development event.

## Approaches
The traveling salesman problem and the vehicle routing problem became the core pillar of the way we approach creating our solution to the challenge statment. When researching about many methods that many would attempt to find the optimum route through a series of location or nodes, the first and natural thing that would be natural to do would be brute force. It can easier in determining the first solution or even fifth solution as the best solution, but for finding the most optimum solution can be near impossibile to find. There may not be the best solution within a person's first hundred tries, they can be closer to find an optimum solution. For the scoper of creating this algorithm , it was not simply computationally feasible to commit to brute force on such as large scale. Because of this, we leant our project to be more heuristic in nature.

Heuristic soltuions felt more doable within a period of three days during the hackathon, because of that it does require perfection. Rather, it requires obtaining the most optimal solution give the amount and calculations that are needed to be done. 

## Running
> pip install streamlit folium streamlit-folium numpy shapely plotly ortools
> streamlit run main.py


