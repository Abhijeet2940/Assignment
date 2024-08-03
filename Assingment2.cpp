#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <algorithm>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

// Base class for drone operations
class DroneOperation {
public:
    virtual void execute() = 0;
};

class Takeoff : public DroneOperation {
public:
    void execute() override {
        std::cout << "Drone is taking off..." << std::endl;
    }
};

class Survey : public DroneOperation {
public:
    void execute() override {
        std::cout << "Drone is surveying the area..." << std::endl;
    }
};

class ReturnToHome : public DroneOperation {
public:
    void execute() override {
        std::cout << "Drone is returning to home..." << std::endl;
    }
};

class Land : public DroneOperation {
public:
    void execute() override {
        std::cout << "Drone is landing..." << std::endl;
    }
};

class Failure : public DroneOperation {
public:
    void execute() override {
        std::cout << "Drone operation failed!" << std::endl;
    }
};

class MissionPlanning {
private:
    std::unordered_map<int, std::vector<std::pair<int, int>>> adjList; // node -> [(neighbor, weight)]
    std::mutex mtx;
    std::condition_variable cv;
    std::atomic<bool> found;
    std::atomic<int> best;
    std::atomic<int> meeting_node;

    void forwardSearch(int src, std::unordered_map<int, int>& dist_f, std::unordered_map<int, int>& parent_f, 
                       std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<>>& pq_f) {
        while (!pq_f.empty()) {
            int u_f = pq_f.top().second;
            pq_f.pop();

            for (const auto& neighbor : adjList[u_f]) {
                int v = neighbor.first;
                int weight = neighbor.second;

                if (dist_f[u_f] + weight < dist_f[v]) {
                    dist_f[v] = dist_f[u_f] + weight;
                    pq_f.push({dist_f[v], v});
                    parent_f[v] = u_f;
                }

                if (dist_b[v] != std::numeric_limits<int>::max() && dist_f[v] + dist_b[v] < best) {
                    best = dist_f[v] + dist_b[v];
                    meeting_node = v;
                    found = true;
                    cv.notify_all();
                }
            }
        }
    }

    void backwardSearch(int dst, std::unordered_map<int, int>& dist_b, std::unordered_map<int, int>& parent_b, 
                        std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<>>& pq_b) {
        while (!pq_b.empty()) {
            int u_b = pq_b.top().second;
            pq_b.pop();

            for (const auto& neighbor : adjList[u_b]) {
                int v = neighbor.first;
                int weight = neighbor.second;

                if (dist_b[u_b] + weight < dist_b[v]) {
                    dist_b[v] = dist_b[u_b] + weight;
                    pq_b.push({dist_b[v], v});
                    parent_b[v] = u_b;
                }

                if (dist_f[v] != std::numeric_limits<int>::max() && dist_f[v] + dist_b[v] < best) {
                    best = dist_f[v] + dist_b[v];
                    meeting_node = v;
                    found = true;
                    cv.notify_all();
                }
            }
        }
    }

public:
    void addEdge(int u, int v, int weight) {
        std::lock_guard<std::mutex> lock(mtx);
        adjList[u].push_back({v, weight});
        adjList[v].push_back({u, weight});
    }

    std::vector<int> bidirectionalDijkstra(int src, int dst) {
        std::unordered_map<int, int> dist_f, dist_b;
        std::unordered_map<int, int> parent_f, parent_b;
        std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<>> pq_f, pq_b;

        for (const auto& pair : adjList) {
            dist_f[pair.first] = std::numeric_limits<int>::max();
            dist_b[pair.first] = std::numeric_limits<int>::max();
        }

        pq_f.push({0, src});
        dist_f[src] = 0;
        parent_f[src] = -1;

        pq_b.push({0, dst});
        dist_b[dst] = 0;
        parent_b[dst] = -1;

        found = false;
        best = std::numeric_limits<int>::max();
        meeting_node = -1;

        std::thread forwardThread(&MissionPlanning::forwardSearch, this, src, std::ref(dist_f), std::ref(parent_f), std::ref(pq_f));
        std::thread backwardThread(&MissionPlanning::backwardSearch, this, dst, std::ref(dist_b), std::ref(parent_b), std::ref(pq_b));

        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this] { return found.load(); });

        forwardThread.join();
        backwardThread.join();

        std::vector<int> path;
        if (meeting_node == -1) return path; // No path found

        for (int v = meeting_node; v != -1; v = parent_f[v])
            path.push_back(v);
        std::reverse(path.begin(), path.end());
        for (int v = parent_b[meeting_node]; v != -1; v = parent_b[v])
            path.push_back(v);

        return path;
    }
};

void performOperation(DroneOperation* operation) {
    operation->execute();
}

int main() {
    MissionPlanning missionPlanning;
    
    // Creating a fully connected graph with 100 nodes
    for (int i = 0; i < 100; ++i) {
        for (int j = i + 1; j < 100; ++j) {
            missionPlanning.addEdge(i, j, rand() % 10 + 1);
        }
    }

    int source = 0;
    int destination = 99;

    Takeoff takeoff;
    Survey survey;
    ReturnToHome returnToHome;
    Land land;
    Failure failure;

    std::vector<DroneOperation*> operations = {&takeoff, &survey, &returnToHome, &land};

    std::vector<std::thread> threads;
    for (DroneOperation* operation : operations) {
        threads.push_back(std::thread(performOperation, operation));
    }

    for (std::thread& t : threads) {
        t.join();
    }

    std::cout << "Planning mission from node " << source << " to node " << destination << "..." << std::endl;
    std::vector<int> path = missionPlanning.bidirectionalDijkstra(source, destination);
    if (path.empty()) {
        failure.execute();
    } else {
        std::cout << "Optimal path found: ";
        for (int node : path) {
            std::cout << node << " ";
        }
        std::cout << std::endl;
        survey.execute();
    }

    return 0;
}
