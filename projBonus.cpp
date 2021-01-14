#include <iostream> 
#include <algorithm>
#include <list> 
#include <cmath> 
#include <math.h>
#include <utility>
#include <vector>
#define MAX 1000000.0 
#define INF 10000 
using namespace std; 

// Class to represent points.
class Point {
private:
        double xval, yval;
public:
        // Constructor uses default arguments to allow calling with zero, one,
        // or two values.
        Point(int x = 0.0, int y = 0.0) {
                xval = x;
                yval = y;
        }

        // Extractors.
        int x() { return xval; }
        int y() { return yval; }

        // Distance to another point.  Pythagorean thm.
        int dist(Point other) {
                int xd = xval - other.xval;
                int yd = yval - other.yval;
                return sqrt(xd*xd + yd*yd);
        }

        // Add or subtract two points.
        Point add(Point b)
        {
                return Point(xval + b.xval, yval + b.yval);
        }
        Point sub(Point b)
        {
                return Point(xval - b.xval, yval - b.yval);
        }

        // Move the existing point.
        void move(double a, double b)
        {
                xval += a;
                yval += b;
        }

        // Print the point on the stream.  The class ostream is a base class
        // for output streams of various types.
        void print(ostream &strm)
        {
                strm << "(" << xval << "," << yval << ")";
        }
};

// Print a line of the form x op y = z, where x, y, and z are points. 
void prline(ostream &strm, Point x, char *op, Point y, Point z)
{
        x.print(strm);
        cout << " " << op << " ";
        y.print(strm);
        cout << " = ";
        z.print(strm);
        cout << endl;
}

class Graph 
{ 
private:
    int V;    
    list<int> *adj;  
public: 
    Graph(int V)   { this->V = V; adj = new list<int>[V]; } 
    ~Graph()       { delete [] adj; } 
  
    void addEdge(int v, int w); 
    void removeEdge(int v, int w);
    int getVertex();
    list<int> *getAdj();
  
    void BFS(int s);
    void greedyColoring(); 
}; 

void Graph::BFS(int s){
    bool *visited = new bool[V];
    for(int i=0; i<V; i++)
        visited[i] = false;
    list<int> queue;

    visited[s]=true;
    queue.push_back(s);

    list<int>::iterator i;
    while(!queue.empty()){
        s = queue.front();
        cout << s << " ";
        queue.pop_front();

        for(i=adj[s].begin(); i != adj[s].end(); i++){
            if(!visited[*i]){
                visited[*i] = true;
                queue.push_back(*i);
            }
        }
    }

}

list<int> *Graph::getAdj(){
    return adj;
}
  
int Graph::getVertex(){
    return V;
}

void Graph::addEdge(int v, int w) 
{ 
    adj[v].push_back(w); 
    adj[w].push_back(v);  
} 

void Graph::removeEdge(int v, int w){
    list<int>::iterator it  = std::find(adj[v].begin(), adj[v].end(), w);
    adj[v].remove(*it);
    it  = std::find(adj[w].begin(), adj[w].end(), v);
    adj[w].remove(*it);
}
  
void Graph::greedyColoring() 
{ 
    int result[V]; 
  
    result[0]  = 0; 
    int i=100, j=100, k=100, l=100;
  
    for (int u = 1; u < V; u++) 
        result[u] = -1; 
  
    bool available[V]; 
    for (int cr = 0; cr < V; cr++) 
        available[cr] = false; 
  
    for (int u = 1; u < V; u++) 
    { 
        list<int>::iterator i; 
        for (i = adj[u].begin(); i != adj[u].end(); ++i) 
            if (result[*i] != -1) 
                available[result[*i]] = true; 
  
        int cr; 
        for (cr = 0; cr < V; cr++) 
            if (available[cr] == false) 
                break; 
  
        result[u] = cr;
  
        for (i = adj[u].begin(); i != adj[u].end(); ++i) 
            if (result[*i] != -1) 
                available[result[*i]] = false; 
    }  

    for (int u = 0; u < V; u++){ 
        cout << "Vertex " << u << " --->  Color "
             << result[u] << endl; 
    } 
    for(int s=0; s<V; s++){
        switch(result[s]){
            case 0:i--;
            break;
            case 1:j--;
            break;
            case 2:k--;
            break;
            case 3:l--;
            break;
        }
    }
    for(int s=0; s<V; s++){   
        if(i<=j && i<= k && i<=l){
            if(result[s] == 0){
                cout << "Vertex " << s << endl;
            }
        }
        if(j<i && j< k && j<l){
            if(result[s] == 1){
                cout << "Vertex " << s << endl;
            }
        }
        if(k<i && k < j && k<l){
            if(result[s] == 1){
                cout << "Vertex " << s << endl;
            }
        }
        if(l<i && l < j && l<k){
            if(result[s] == 1){
                cout << "Vertex " << s << endl;
            }
        }
    }
} 

bool onSegment(Point p, Point q, Point r) 
{ 
    if (q.x() <= max(p.x(), r.x()) && q.x() >= min(p.x(), r.x()) && 
            q.y() <= max(p.y(), r.y()) && q.y()>= min(p.y(), r.y())) 
        return true; 
    return false; 
} 

int orientation(Point p, Point q, Point r) 
{ 
    int val = (q.y() - p.y()) * (r.x() - q.x()) - 
              (q.x() - p.x()) * (r.y() - q.y()); 
  
    if (val == 0) return 0;  // colinear 
    return (val > 0)? 1: 2; // clock or counterclock wise 
} 

bool doIntersect(Point p1, Point q1, Point p2, Point q2) 
{ 
    // Find the four orientations needed for general and 
    // special cases 
    int o1 = orientation(p1, q1, p2); 
    int o2 = orientation(p1, q1, q2); 
    int o3 = orientation(p2, q2, p1); 
    int o4 = orientation(p2, q2, q1); 
  
    // General case 
    if (o1 != o2 && o3 != o4) 
        return true; 
  
    // Special Cases 
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1 
    if (o1 == 0 && onSegment(p1, p2, q1)) return true; 
  
    // p1, q1 and p2 are colinear and q2 lies on segment p1q1 
    if (o2 == 0 && onSegment(p1, q2, q1)) return true; 
  
    // p2, q2 and p1 are colinear and p1 lies on segment p2q2 
    if (o3 == 0 && onSegment(p2, p1, q2)) return true; 
  
     // p2, q2 and q1 are colinear and q1 lies on segment p2q2 
    if (o4 == 0 && onSegment(p2, q1, q2)) return true; 
  
    return false; // Doesn't fall in any of the above cases 
}

bool isInsidePoly(Point polygon[], int n, Point p) 
{ 
    for(int i=0; i<sizeof(polygon)/sizeof(polygon[0]); i++){
        cout << polygon[i].x() << " " << polygon[i].y() << endl;
    }
    // There must be at least 3 vertices in polygon[] 
    //if (n < 3)  return false; 
  
    // Create a point for line segment from p to infinite 
    Point extreme = {INF, p.y()}; 
  
    // Count intersections of the above line with sides of polygon 
    int count = 0, i = 0; 
    /*do
    { 
        //int next = (i+1)%n; 
    
        // Check if the line segment from 'p' to 'extreme' intersects 
        // with the line segment from 'polygon[i]' to 'polygon[next]' 
        if (doIntersect(polygon[i], polygon[next], p, extreme)) 
        { 
            // If the point 'p' is colinear with line segment 'i-next', 
            // then check if it lies on segment. If it lies, return true, 
            // otherwise false 
            if (orientation(polygon[i], p, polygon[next]) == 0){
                cout << "isInside" << endl; 
               return onSegment(polygon[i], p, polygon[next]); 
            }
            count++; 
        } 
        i = next; 
    } while (i != 0); 
    */
    //cout << "isInside" << endl;
    // Return true if count is odd, false otherwise 
    return count&1;  // Same as (count%2 == 1) 
} 



Point insidePoint(Point i, Point j){
    float d1 = (i.x()+j.x())/2;
    float d2 = (i.y()+j.y())/2;
    Point inside(d1,d2);
    return inside;

}


float distanceFunc(Point i, Point j){
    float d1 = (i.x() - j.x()) * (i.x() - j.x());
    float d2 = (i.y() - j.y()) * (i.y() - j.y());
    return sqrt(d1+d2);
}

float cosFunc(Point x1, Point x2, Point x3){
    float a = distanceFunc(x2, x3);
    float b = distanceFunc(x1, x3);
    float c = distanceFunc(x1, x2);
    float d = 2 * a * b;
    c= c*c;
    a= a*a;
    b=b*b;
    c= c-b;
    c=c-a;
    c= c/d;
    return acos(c);
       }

bool convex(Point i, Point j , Point k)
{
if (cosFunc(i, k ,j) < 180 && cosFunc(i,k,j) > 0){
    return true;
}
else
    return false;
}

float area2(int x1, int y1, int x2, int y2, int x3, int y3) 
{ 
   return abs((x1*(y2-y3) + x2*(y3-y1)+ x3*(y1-y2))/2.0); 
} 
  
bool isInside(int x1, int y1, int x2, int y2, int x3, int y3, int x, int y) 
{     
   float A = area2(x1, y1, x2, y2, x3, y3);   
   float A1 = area2 (x, y, x2, y2, x3, y3);  
   float A2 = area2 (x1, y1, x, y, x3, y3); 
   float A3 = area2 (x1, y1, x2, y2, x, y); 
   return (A == A1 + A2 + A3); 
} 

bool isEar(vector<pair<Point, int> > vect, int k, int size, Point polygon[]){
    Point prev ;
    Point curr ;
    Point next ;
    Point mid;
    Point in;
    int n;
    if(k == vect[0].second){
        prev = vect[vect.size()-1].first;
        curr = vect[k].first;
        next = vect[k+1].first;
    }
    else if(k==vect.size()){
        prev = vect[k-1].first;
        curr = vect[k].first;
        next = vect[0].first;
    }
    else{
        prev = vect[k-1].first;
        curr = vect[k].first;
        next = vect[k+1].first;
        }
    if(convex(prev, curr, next)){
        //cout << "T1" << endl;
        mid = (insidePoint(prev, next));
        n = sizeof(polygon)/sizeof(polygon[0]);
        if(!isInsidePoly(polygon, n, mid)){
            return false;
        }
        for(int i=0; i<vect.size(); i++){
            if((vect[i].first.x() != prev.x()) && (vect[i].first.x() != curr.x()) && (vect[i].first.x() != next.x()) && (vect[i].first.y() != prev.y()) && (vect[i].first.y() != curr.y()) && (vect[i].first.y() != next.y())){
                in = vect[i].first;
                cout << "T3" << endl;
                if(isInside(prev.x(), prev.y(), curr.x(), curr.y(), next.x(), next.y(), in.x(), in.y())){
                    cout << "T4" << endl;
                    return false;
                }  
            }
        }
    }
    return true;
}

void triangulate(vector<pair<Point, int> > vect, vector<pair<Point, int> > vectC, Graph &g1, Graph &g2, int size, Point polygon[]){
    while(vectC.size()>3){
        for(int i=0; i<vectC.size(); i++){
            if(vectC.size()>3){
                if(isEar(vectC, i, size, polygon)){
                    if(i == 0){
                        g1.addEdge(vectC[vectC.size()].second, vectC[i+1].second);
                        g2.addEdge(vectC[vectC.size()].second, vectC[i+1].second);
                        g2.removeEdge(vectC[vectC.size()].second, vectC[i].second);
                        g2.removeEdge(vectC[i].second, vectC[i+1].second);
                        vectC.erase(vectC.begin()+i);
                        i--;
                        cout << "Workd" << endl;
                    }
                    else if(i==vectC.size() || i > vectC.size()){
                        g1.addEdge(vectC[i-1].second, vectC[0].second);
                        g2.addEdge(vectC[i-1].second, vectC[0].second);
                        g2.removeEdge(vectC[i-1].second, vectC[i].second);
                        g2.removeEdge(vectC[i].second, vectC[0].second);
                        vectC.erase(vectC.begin()+i);
                        i--;
                        cout << "Worke" << endl;
                    }
                    else{
                        g1.addEdge(vectC[i-1].second, vectC[i+1].second);
                        g2.addEdge(vectC[i-1].second, vectC[i+1].second);
                        g2.removeEdge(vectC[i-1].second, vectC[i].second);
                        g2.removeEdge(vectC[i].second, vectC[i+1].second);
                        vectC.erase(vectC.begin()+i);
                        i--;
                        cout << "Work" << endl;
                    }
                }
            }
        }
    }
    g1.addEdge(vectC[0].second, vectC[1].second);
    g1.addEdge(vectC[1].second, vectC[2].second);
    g1.addEdge(vectC[2].second, vectC[0].second);
}

int main() 
{ 
    int vex;
    int xx;
    int yy;


    cout << "Insert amount of Vertices: ";
    cin >> vex;

    //Creating Graphs
    Graph g1(vex); 
    Graph g2(vex);
    for(int i=0; i<vex; i++){
        if(i<vex-1){
            g1.addEdge(i, i+1); 
            g2.addEdge(i, i+1);
        }
        else if(i==vex-1){
            g1.addEdge(i, 0);
            g2.addEdge(i, 0);
        }
    }

    //Creating Points for Graphs
    Point p[vex];
    cout << "Insert vertex of shape(x,y) in order: " << endl;
    for(int i=0; i<vex; i++){
        cout << "Vertex " << i << ": ";
        cin >> xx >> yy;
        Point point(xx,yy);
        p[i]=point;
    }

    //Number to keep track of vertices
    int x[vex];
    for(int i=0; i<vex; i++){
        x[i] = i;
    }

    vector< pair<Point, int> > vert;
    vector< pair<Point, int> > vert2;
    
    for(int i=0; i<vex; i++){
        vert.push_back(make_pair(p[i], x[i]));
    }

    for(int i=0; i<vex; i++){
        vert2.push_back(make_pair(p[i], x[i]));
    }
    cout << endl;
    triangulate(vert, vert2, g1, g2, vex, p);
    g1.greedyColoring();

    return 0; 
} 