
#include <iostream>
#include <CImg.h>
#include "Graph.h"

using namespace cimg_library;

void drawGraph(std::vector<long unsigned int> path, const unsigned char color[],CImg<unsigned char> &image,CImgDisplay &mainDisplay);

int main(){
    
    Graph test("moonglade2.map");
    
    test.generateBoostGraph();
    
    /*THERE ARE SLIGHT INCONSISTENCIES IN THE OUTPUT DRAWING AS THE MAPFILE IS NOT COMPLETELY ACCURATE
    **TO THE IMAGE FILE-- EVEN WHEN IMAGE IS SCALED OR CROPPED TO ACCOUNT FOR IT 
    */

    const unsigned char dijkcolor[] = {0, 0, 255}; //blue
    const unsigned char astarcolor[] = {255, 0, 0}; // red
    const unsigned char bnbColor[] = {0, 180, 180}; //bluegreen
    const unsigned char bnbEColor[] = {100, 80, 180}; //darkblue
    const unsigned char bnbHColor[] = {200, 180, 80}; //yellow

    CImg<unsigned char> image("moongladeScale9.jpg"), visu(512,512,3,1);
    CImgDisplay mainDisplay(image, "MoonGlade");

    drawGraph(test.BranchAndBound(107,130,332,74), bnbColor,image,mainDisplay);
    std::cout<< "Drew branch and bound path" <<std::endl;
    drawGraph(test.BranchAndBoundExtended(107,130,332,74), bnbEColor,image,mainDisplay);
    std::cout<< "Drew branch and bound extended path" <<std::endl;
    drawGraph(test.BranchAndBoundHeuristic(107,130,332,74), bnbHColor,image,mainDisplay);
    std::cout<< "Drew branch and bound with heuristic path" <<std::endl;
    drawGraph(test.AStar(107,130,332,74), astarcolor,image,mainDisplay);
    std::cout << "Drew Astar path" << std::endl;
    drawGraph(test.Dijkstras(107,130,332,74), dijkcolor,image,mainDisplay);
    std::cout<< "Drew Dijkstra path" <<std::endl;
    //allow time for each algorithm to execute and draw

    while(!mainDisplay.is_closed());
}

void drawGraph(std::vector<long unsigned int> path, const unsigned char color[] , CImg<unsigned char> &image,CImgDisplay &mainDisplay){

        for (int i = 0; i < path.size(); i++){

            int x = path[i]%512;
            int y = path[i]/512;
          
            image.draw_point(x, y, color);
            image.display(mainDisplay);
        }
}