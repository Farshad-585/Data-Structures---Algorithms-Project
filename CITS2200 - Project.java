import java.util.ArrayList;
import java.util.LinkedList;

/**
 * @author Farshad Ghanbari (21334883)
 * Submission Date: 20/05/2021
 * Each question has a dedicated class for readability.
 */

@SuppressWarnings("unchecked")
public class MyProject implements Project {

    /**
     * Checks whether each device in the network can send a packet to every
     * other device in the network.
     * @param adjlist The structure of the network
     * @return boolean indicating whether all of the devices in the network are 
     * connected to the network
     */
    public boolean allDevicesConnected(int[][] adjlist) {  
        AllDevicesConnected o = new AllDevicesConnected(adjlist);
        return o.addDevicesConnected();
    }

    /**
     * Checks number of paths exist between source and destination
     * @param adjlist The structure of the network
     * @param src The device id of the transmitting device
     * @param dst The device id of the receiving device
     * @return The number of possible different paths in the network that a 
     * packet may take from the transmitting to receiving device
     */
    public int numPaths(int[][] adjlist, int src, int dst) {
        NumPaths o = new NumPaths(adjlist, src, dst);
        return o.numPaths();
    }

    /**
     * For each query tries to find the closes subnet to the source
     * @param adjlist The structure of the network
     * @param addrs An array of IP addresses such that device id i has address 
     * addrs[i]
     * @param src The device id of the transmitting device
     * @param queries An array of queries where each query is a subnet prefix
     * @return int[] of number of hops required to reach each subnet from src
     */
    public int[] closestInSubnet(
                int[][] adjlist, short[][] addrs, int src, short[][] queries) {
        ClosestInSubnet o = new ClosestInSubnet(adjlist, addrs, src, queries);
        return o.closestInSubnet();
    }

    /**
     * Computes the maximum possible download speed from a transmitting device 
     * to a receiving device.
     * @param adjlist The structure of the network
     * @param speeds The maximum speed of each link in the network
     * @param src The device id of the transmitting device
     * @param dst The device id of the receiving device
     * @return The maximum download speed from the transmitting device to the 
     * receiving device
     */
    public int maxDownloadSpeed(int[][] adjlist, int[][] speeds, int src, int dst) {
        // constructing a simpler version of the graph which can incorporate
        // links' capacity and flow easily. O(D + L)
        SpeedGraph speedG = new SpeedGraph(adjlist.length);
        for (int i = 0; i < adjlist.length; i++){
            for (int j = 0; j < adjlist[i].length; j++){
                Link l = new Link(i, adjlist[i][j], speeds[i][j]);
                speedG.addLink(l);
            }
        }
        MaxDownloadSpeed o = new MaxDownloadSpeed(speedG, src, dst); 
        return o .maxSpeed;
    }
    
/******************************************************************************
 *                               Private Classes                              *
 ******************************************************************************/
//     Question 1
//-----------------------------------------------------------------------------/
    /**
     * This class checks whether each device in the network can send a packet 
     * to every other device in the network.
     */
    private class AllDevicesConnected {
        private int[][] adjlist;                // Given adjlist graph
        private int numDevices;                 // Number of devices
        private boolean[] visited;              // Keeps track of device visits
        private ArrayList<Integer>[] reversed;  // Graph direction reversed

        /**
         * Initialises the private fields. 
         * @param adjlist The structure of the network
         */
        private AllDevicesConnected(int[][] adjlist){
            this.adjlist = adjlist;
            this.numDevices = adjlist.length;
            this.visited = new boolean[numDevices];
            this.reversed = inverse(adjlist);
        }

        /**
         * Performs the main logic. Starts from index 0 and tries to visit all
         * the devices in the network in the original direction. If any device
         * not visited, devices are not connected. If devices are connected in
         * the original direction, we check the reverse direction. If any device 
         * in the reverse direction is not visited, devices are not connected.
         * @return true if devices are connected, false if not.
         */
        private boolean addDevicesConnected(){
            dfs(adjlist, 0);
            if (!connected()) return false;
            visited = new boolean[numDevices];
            dfs(reversed, 0);
            if (!connected()) return false;
            return true;
        }
        
        /**
         * Runs through the visited array and checks to see if each device has
         * been visited.
         * @return true if all visited, false if not
         */
        private boolean connected(){
            for (boolean visit : visited){
                if (!visit){
                    return false;
                }
            }
            return true;
        }

        /**
         * Perform a depth first search of the original graph.
         * @param adjlist original graph
         * @param d index to start from, 0 in this case
         */
        private void dfs(int[][] adjlist, int d){
            visited[d] = true;
            for (int l : adjlist[d]){
                if (!visited[l]){
                    dfs(adjlist, l);
                }
            }
        }

        /**
         * Perform a depth first search of the reversed graph
         * @param inverse reversed graph
         * @param d index to start from, 0 in this case
         */
        private void dfs(ArrayList<Integer>[] inverse, int d){
            visited[d] = true;
            for (int l : inverse[d]){
                if (!visited[l]){
                    dfs(inverse, l);
                }
            }
        }
        
        /**
         * Reverses the direction of the links in the given graph
         * @param adjlist original graph
         * @return graph with reversed link directions
         */
        private ArrayList<Integer>[] inverse(int[][] adjlist){
            ArrayList<Integer>[] inverse = 
                            (ArrayList<Integer>[]) new ArrayList[numDevices];
            for (int d = 0; d < numDevices; d++){
                inverse[d] = new ArrayList<>();
            }
            for (int d = 0; d < numDevices; d++){
                for (int l : adjlist[d]){
                    inverse[l].add(d);
                }
            }
            return inverse;
        }
    }
/******************************************************************************/ 
//     Question 2
//-----------------------------------------------------------------------------/    
    /**
     * This class computes number of paths exisitng between a source device and 
     * a destination device.
     */
    private class NumPaths {
        private int[][] adjlist;            // Given graph
        private int src;                    // Source
        private int dst;                    // Destination
        private int numDevices;             // Number of devices in network
        private boolean visited[];          // Keeps track of device visits
        private int pathCount;              // Total paths counted

        /**
         * Initialises the private fields.
         * @param adjlist The structure of the network
         * @param src The device id of the transmitting device
         * @param dst The device id of the receiving device
         */
        private NumPaths(int[][] adjlist, int src, int dst){
            this.adjlist = adjlist;
            this.src = src;
            this.dst = dst;
            this.numDevices = adjlist.length;
            this.visited = new boolean[numDevices];
        }

        /**
         * Performs the path count via a depth first search.
         * @return number of paths counted.
         */
        private int numPaths(){
            if (src == dst) return 1;
            dfs(adjlist, src, dst);
            return pathCount;
        }

        /**
         * Path count is incremented whenever depth first search arrives at the
         * destination during its recursion.
         * @param adjlist given graph
         * @param src srouce
         * @param dst destination
         */
        private void dfs(int[][] adjlist, int src, int dst){
            if (src == dst){
                pathCount++;
            } else {
                // We have visited the source
                visited[src] = true;
                // Lets check all its connections
                for (int l : adjlist[src]){
                    if (!visited[l]){
                        // This connection has not been visited yet. So lets 
                        // visit it.
                        dfs(adjlist, l, dst);
                        // When dfs returns from its recursion or the path it
                        // checked, we put 'not visited' for the current vertex
                        // so that we can check its other paths later.
                        visited[l] = false;
                    }
                }
            }
        }
    }

/******************************************************************************/ 
//     Question 3
//-----------------------------------------------------------------------------/
    /**
     * A class that for each query, compute the minimum number of hops required 
     * to reach some device in the specified subnet
     */
    private class ClosestInSubnet {
        private int[][] adjlist;            // Given graph
        private short[][] addrs;            // Ip addresses
        private short[][] queries;          // List of queries
        private int src;                    // Source device
        private int numDevices;             // Number of devices in the network
        private int lenQueries;             // Number of queries
        private int[] hops;                 // Hops array to be returned
        private int[] parent;               // Parent of each device visited
        boolean[] visited;

        /**
         * Initialises the private fields.
         * @param adjlist The structure of the network
         * @param addrs An array of IP addresses such that device id i has 
         * address addrs[i]
         * @param src The device id of the transmitting device
         * @param queries An array of queries where each query is a subnet prefix
         */
        private ClosestInSubnet(
                int[][] adjlist, short[][] addrs, int src, short[][] queries){
            this.adjlist = adjlist;
            this.addrs = addrs;
            this.queries = queries;
            this.src = src;
            this.numDevices = adjlist.length;
            this.lenQueries = queries.length;
            this.hops = new int[lenQueries];
            this.parent = new int[numDevices];
            
        }

        /**
         * First it performs a breadth first search of the network. This step
         * finds a parent for each vertex in a way that if we follow the parents
         * we arrive at the source device in minimum amount of hops.
         * @return int[] containing number of hops for each query
         */
        private int[] closestInSubnet() {
            bfs();
            for (int i = 0; i < lenQueries; i++){
                // If no query, skip it.
                if (queries[i].length == 0){
                    continue;
                }
                calculateHops(i);
            }
            return hops;
        }

        /**
         * Starts at the source device. Scans all its linked devices and puts 
         * them in a queue. This is continued until there is no device left 
         * in the queue. The device removed from the queue in each step will
         * be the parent of the devices it links to.
         */
        private void bfs(){
            LinkedList<Integer> q = new LinkedList<>();
            visited = new boolean[numDevices];
            parent[src] = -1;
            q.add(src);
            visited[src] = true; 

            while (!q.isEmpty()){
                int u = q.removeFirst();
                for (int l : adjlist[u]){
                    if (!visited[l]){
                        parent[l] = u;
                        visited[l] = true;
                        q.add(l);
                    }
                }
            }
        }

        /**
         * Given the query, it calcules number of hops required to arrive to
         * the closest subnet in the query from the source.
         * @param query index of query
         */
        private void calculateHops(int query){
            // We scan through devices in the network
            for (int i = 0; i < numDevices; i++){
                // If bfs has not been able to reach this device, we skip it.
                if (!visited[i]){
                    continue;                  
                }
                // If this device is in the subnet, we increment hops until
                // we reach the source.
                if (isSubnet(addrs[i], queries[query])){
                    int p = parent[i];
                    while ( p != -1){
                        hops[query]++;
                        p = parent[p];
                    }
                    break;
                }
                // If we are looking at the final device and we haven't found 
                // a subnet yet, subnet cannot be reached. 
                if (i == numDevices - 1) {
                    hops[query] = Integer.MAX_VALUE;
                    break;
                }
            }
        }
        
        /**
         * Checks whether ip is in the subnet of query
         * @param addr ip address
         * @param query subnet in query
         * @return true if in subnet, false if not.
         */
        private boolean isSubnet(short[] addr, short[] query){
            for (int i = 0; i < query.length; i++){
                if (query[i] != addr[i]){
                    return false;
                }
            }
            return true;
        }
    }
/******************************************************************************/ 
//     Question 4
//-----------------------------------------------------------------------------/
    /**
     * A class that uses a modification of Ford-Fulkerson algorithm suggested
     * by J. Edmonds and R.Karp. In this approach an augmenting path for 
     * the flow of download is found through a breadth first search.
     *  The steps of the logic are outlined in below.
     */
    private class MaxDownloadSpeed {
        private int numDevices;     // Number of devices in the network
        private boolean[] visited;  // Keeps track of visited devices
        private Link[] linkTo;      // Links array showing parent of each vertex
        private int maxSpeed;       // Max download speed calculated

        /**
         * Calculates max download speed possible for the source device to send
         * data to the destination device.
         * @param g given graph
         * @param src source device
         * @param dst destination device
         */
        private MaxDownloadSpeed(SpeedGraph g, int src, int dst){
            this.numDevices = g.numDevices;
            if (src == dst){
                maxSpeed = -1;
            } else {
                // We keep searching for an augmenting path in the 
                // residual network. If there is another path that data can 
                // flow, we add to maxdl speed.
                while (hasAugmentingPath(g, src, dst)){
                    // We have a path. We start from the destination device and 
                    // go back to the source device in the path given. 
                    // We compare all links in this path and find out which  
                    // link has the lowest capacity. The capacity of that  
                    // link will be the bottleneck flow value for this path.
                    int bottleNeck = Integer.MAX_VALUE;
                    for (int v = dst; v!= src; v = linkTo[v].other(v)){
                        bottleNeck = Math.min(bottleNeck, 
                                                linkTo[v].residualSpeedTo(v));
                    }
                    
                    // Then we need to reconstruct the residual graph and
                    // and augment the flow . The speed of each link in the 
                    // path is incremented by the bottleneck calculated above.
                    for (int v = dst; v != src; v = linkTo[v].other(v)){
                        linkTo[v].addSpeedTo(v, bottleNeck);
                    }
                    // We found a new path. Maximum download speed found sofar
                    // should be increased by the new path bottleneck value.
                    maxSpeed += bottleNeck;
                }
            }
        }

        /**
         * This method implements a breadth first search to find an augmenting
         * path from a source device to the destination device. An augmenting 
         * path exists only when there is residual capacity left in the path for
         * data to flow through.
         * @param g given graph
         * @param src source device
         * @param dst destination device
         * @return true if there is a path, false if not
         */
        private boolean hasAugmentingPath(SpeedGraph g, int src, int dst){
            // Reset the links and visited array from previous round. We still
            // have the data of the residual network and how much capacity left
            // in each link.
            linkTo = new Link[numDevices];
            visited = new boolean[numDevices];

            LinkedList<Integer> q = new LinkedList<>();
            // Start from the source device
            q.add(src);
            visited[src] = true;
            // Until there is a device in the path, the queue will be full
            // We stop if we reach destination or if queue is empty
            while (!q.isEmpty() && !visited[dst]){
                int from = q.removeFirst();
                // Check all the links connected to the current device
                for (Link l : g.adj(from)){
                    // link 'l' is from 'from' device to 'to' device
                    int to = l.other(from);
                    // If there is residual capacity left in this link to next 
                    // device and we havent visited the next device, 
                    // it should be in the path.
                    if (l.residualSpeedTo(to) > 0 && !visited[to]){
                        linkTo[to] = l;
                        visited[to] = true;
                        q.add(to);
                    }
                }
            }
            // If we have visited the destination device, there has been an
            // augmenting path to it.
            return visited[dst];
        }
    }

    /**
     * A class representing the adjlist graph given in the question. 
     * Keeping track of links flow and capacity is easier in this graph. 
     * It also easier to represent the residual network.
     */
    private class SpeedGraph {
        private int numDevices;        // Number of devices in the network.
        private ArrayList<Link>[] adj; // Each device has an arraylist of links

        /**
         * Constructor to initialise fields.
         * @param numDevices Number of devices in the network.
         */
        private SpeedGraph(int numDevices){
            this.numDevices = numDevices;
            adj = (ArrayList<Link>[]) new ArrayList[numDevices];
            for (int i = 0; i < numDevices; i++){
                adj[i] = new ArrayList<Link>();
            }
        }

        /**
         * Adds a new link between the respective devices.
         * @param l
         */
        private void addLink(Link l){
            int from = l.from();
            int to = l.to();
            adj[from].add(l);
            adj[to].add(l);
        }

        /**
         * Iterable to use a for each loop
         * @param device the device to look for its links
         * @return returns the arraylist of links for that device.
         */
        private Iterable<Link> adj(int device){
            return adj[device];
        }
    }

    /**
     * A link classes representing the links between each device.
     */
    private class Link{
        private int from;           // Links from device
        private int to;             // Link to device
        private int allowedSpeed;   // Capacity
        private int currSpeed;      // Current flow

        /**
         * Constructor to initialise fields
         * @param from from device
         * @param to to device
         * @param allowedSpeed Capacity of link
         */
        private Link(int from, int to, int allowedSpeed){
            this.from = from;
            this.to = to;
            this.allowedSpeed = allowedSpeed;
            this.currSpeed = 0;
        }

        /**
         * @return device links from
         */
        private int from(){
            return from;
        }

        /**
         * @return device links to
         */
        private int to(){
            return to;
        }

        /**
         * What is the other end of this link
         * @param device Source end of link
         * @return Destination end of the link
         */
        private int other(int device){
            if (device == from) return to;
            else return from;
        }

        /**
         * How much residual flow/speed left to be used in this link. 
         * In the residual network links can be represented both ways.
         * @param device the destination of the link
         * @return the residual flow/speed left in the link
         */
        private int residualSpeedTo(int device){
            if (device == from) return currSpeed;
            else return allowedSpeed - currSpeed;
        }

        /**
         * Add flow/more speed to this link equivalent to the bottleneck 
         * value given.
         * @param device Destination of the link
         * @param bottleNeck bottleneck value
         */
        private void addSpeedTo(int device, int bottleNeck){
            if (device == from) currSpeed -= bottleNeck;
            else if (device == to) currSpeed += bottleNeck;
        }
    }
}
//---------------------------------The End--------------------------------------
