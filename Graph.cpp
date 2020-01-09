#include "Graph.h"


Graph::Graph(std::string fileName){
    this->fileName = fileName;
    populateGraphVector();

}

void Graph::populateGraphVector(){
    charMap.resize(512);
    std::ifstream inFile;
    std::string line;
    inFile.open(this->fileName);
    int i = 0;
    while (getline(inFile, line)) {
        //
    // store each line in the vector
    if (line.size() == 512){
        
        for (auto &myChar : line){
            charMap[i].push_back(myChar);
        }
        i++;

    }
    
    }
    inFile.close();
}
void Graph::printCharMap(){
    for (auto &line :charMap){
        for (auto &myChar : line){
            std::cout << myChar;
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
    std::cout << charMap[3][0] << std::endl;
    std::cout << charMap[4][0] << std::endl;
}

void Graph::generateBoostGraph(){
    
    for (int i = 0; i <  512; i++){
        for(int j = 0; j < 512; j++){
            boost::add_vertex(adjList);
            int index = (i*512) + j;
            adjList[index].x = i;
            adjList[index].y = j;

           
            

            
            //[i] row
            //[j] col
            
        } 
    } //CREATING VERTICES

    
    for (int i = 0; i <  512; i++){
        for(int j = 0; j < 512; j++){
            //[i] row
            //[j] col
               //test left
               
               if (j > 0){
                   if (charMap[i][j-1] == '.' ){
                   boost::add_edge((i*512)+j,(i*512)+j-1,1,adjList);
                   }
               }
               //test right
               if (j < 511){
                   if (charMap[i][j+1] == '.' ){
                   boost::add_edge((i*512)+j,(i*512)+j+1,1,adjList);
                   }
               }
               //test up
               if (i > 0){
                   if (charMap[i-1][j] == '.' ){
                   boost::add_edge((i*512)+j,((i-1)*(512)+j),1,adjList);
                   }
               }

               //test down
               if (i < 511){
                   if (charMap[i+1][j] == '.' ){
                   boost::add_edge((i*512+j),((i+1)*(512)+j),1,adjList);
                   }
               }
               //test leftup
               if (j > 0 && i > 0){
                   if (charMap[i-1][j-1] == '.' ){
                   boost::add_edge((i*512)+j,(i-1)*(512)+j-1,1,adjList);
                   }
               }
               //test leftdown
               if (j > 0 && i < 511){
                   if (charMap[i+1][j-1] == '.' ){
                   boost::add_edge((i*512)+j,(i+1)*(512)+j-1,1,adjList);
                   }
               }
               //test rightup
               if (j < 511 && i > 0){
                   if (charMap[i-1][j+1] == '.' ){
                   boost::add_edge((i*512)+j,(i-1)*(512)+j+1,1,adjList);
                   }
               }
               //test rightdown
               if (j < 511 && i < 511){
                   if (charMap[i+1][j+1] == '.' ){
                   boost::add_edge((i*512)+j,((i+1)*(512)+j+1),1,adjList);
                   }
               } 
               
        } 
    } //ADDING NEIGHBORS
   
}
std::vector<long unsigned int> Graph::BranchAndBound(int startX, int startY, int endX, int endY){
    vertex_t startNode = (startY*512) + startX;
    vertex_t endNode = (endY*512)+ endX;

    std::vector<vertex_t> p(boost::num_vertices(adjList));
    std::vector<int> d(boost::num_vertices(adjList));
    std::vector<Vertex> predecessors(boost::num_vertices(adjList));
    IndexMap indexmap = boost::get(boost::vertex_index, adjList);
    WeightMap weightmap = boost::get(boost::edge_weight_t(), adjList);
    
    try{
     astar_search (adjList, startNode,
        emptyHeuristic<mygraph_t, cost>
        (adjList, endNode),
       boost::predecessor_map(&p[0]).
       visitor(astar_goal_visitor<vertex_t>(endNode)));
    }  catch(found_goal fg){

      std::vector< vertex_t > path;
      path.reserve(500);
      vertex_t current = endNode;
        
        
      while(current!=startNode) {
          
          path.push_back(current);
          current=p[current];
          
      }
      
      path.push_back(startNode);
      
      return path;

}
}

std::vector<long unsigned int> Graph::BranchAndBoundExtended(int startX, int startY, int endX, int endY){
    vertex_t startNode = (startY*512) + startX;
    vertex_t endNode = (endY*512)+ endX;

    std::vector<vertex_t> p(boost::num_vertices(adjList));
    std::vector<int> d(boost::num_vertices(adjList));
    std::vector<Vertex> predecessors(boost::num_vertices(adjList));
    IndexMap indexmap = boost::get(boost::vertex_index, adjList);
    WeightMap weightmap = boost::get(boost::edge_weight_t(), adjList);
    
    try{
     astar_search (adjList, startNode,
        emptyHeuristic<mygraph_t, cost>
        (adjList, endNode),
       boost::predecessor_map(&p[0]).distance_map(&d[0]).
       visitor(astar_goal_visitor<vertex_t>(endNode)));
    }  catch(found_goal fg){

      std::vector< vertex_t > path;
      path.reserve(500);
      vertex_t current = endNode;
        
        
      while(current!=startNode) {
          
          path.push_back(current);
          current=p[current];
          
      }
      

      path.push_back(startNode);
     
      return path;

}
}

std::vector<long unsigned int> Graph::BranchAndBoundHeuristic(int startX, int startY, int endX, int endY){
    vertex_t startNode = (startY*512) + startX;
    vertex_t endNode = (endY*512)+ endX;

    std::vector<vertex_t> p(boost::num_vertices(adjList));
    std::vector<int> d(boost::num_vertices(adjList));
    std::vector<Vertex> predecessors(boost::num_vertices(adjList));
    IndexMap indexmap = boost::get(boost::vertex_index, adjList);
    WeightMap weightmap = boost::get(boost::edge_weight_t(), adjList);
    
    try{
     astar_search (adjList, startNode,
        distance_heuristic<mygraph_t, cost>
        (adjList, endNode),
       boost::predecessor_map(&p[0]).
       visitor(astar_goal_visitor<vertex_t>(endNode)));
    }  catch(found_goal fg){


      std::vector< vertex_t > path;
      path.reserve(500);
      vertex_t current = endNode;
        
        
      while(current!=startNode) {
          
          path.push_back(current);
          current=p[current];
          
      }
      

      path.push_back(startNode);
     
      return path;

}
}

std::vector<long unsigned int> Graph::AStar(int startX, int startY, int endX, int endY){
    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    vertex_t startNode = (startY*512) + startX;
    vertex_t endNode = (endY*512)+ endX;
    //start and end nodes@@@@@@@@@@@@@@@@@@@@@@@@@@@
    //
    
    std::vector<vertex_t> p(boost::num_vertices(adjList));
    std::vector<int> d(boost::num_vertices(adjList));
    std::vector<Vertex> predecessors(boost::num_vertices(adjList));
    IndexMap indexmap = boost::get(boost::vertex_index, adjList);
    WeightMap weightmap = boost::get(boost::edge_weight_t(), adjList);
    try{
     astar_search (adjList, startNode,
       distance_heuristic<mygraph_t, cost>
        (adjList, endNode),
       boost::predecessor_map(&p[0]).distance_map(&d[0]).
       visitor(astar_goal_visitor<vertex_t>(endNode)));
    }  catch(found_goal fg){

      

      std::vector< vertex_t > path;
      path.reserve(500);
      vertex_t current = endNode;
        
        
      while(current!=startNode) {
          
          path.push_back(current);
          current=p[current];
          
      }
      

      path.push_back(startNode);
     

      return path;


    }
}


std::vector<long unsigned int> Graph::Dijkstras(int startX, int startY, int endX, int endY){

    

    
    //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    vertex_t startNode = (startY*512) + startX;
    vertex_t endNode = (endY*512)+ endX;
    //start and end nodes@@@@@@@@@@@@@@@@@@@@@@@@@@@
    
    
    std::vector<vertex_t> p(boost::num_vertices(adjList));
    std::vector<int> d(boost::num_vertices(adjList));
    std::vector<Vertex> predecessors(boost::num_vertices(adjList));
    IndexMap indexmap = boost::get(boost::vertex_index, adjList);
    WeightMap weightmap = boost::get(boost::edge_weight_t(), adjList);
    boost::dijkstra_shortest_paths(adjList, startNode, &p[0], &d[0], weightmap, indexmap,std::less<int>(), boost::closed_plus<int>(), (std::numeric_limits<int>::max)(), 0,
                              boost::default_dijkstra_visitor());
     

      

      std::vector< vertex_t > path;
      path.reserve(500);
      vertex_t current = endNode;
        
        
      while(current!=startNode) {
          
          path.push_back(current);
          current=p[current];
          
      }
      

      path.push_back(startNode);
    
      return path;
   
      
}