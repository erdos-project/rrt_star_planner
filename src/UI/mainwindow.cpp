#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <map>
#include <iostream>
#include <ctime>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    renderArea = ui->renderArea;
    rrt = renderArea->rrt;
    simulated = false;
}

/**
 * @brief Start the simulator.
 */
void MainWindow::on_startButton_clicked()
{
    if (simulated) {
        ui->statusBox->setText(tr("Please reset!"));
        renderArea->update();
        return;
    }
    simulated = true;
    // get step size and max iterations from GUI.
    rrt->setMaxIterations(ui->maxIterations->text().toInt());
    rrt->setStepSize(ui->stepSize->text().toInt());

    assert(rrt->step_size > 0);
    assert(rrt->max_iter > 0);

    // RRT Algorithm
    clock_t start;
    double duration;
    start = clock();

    map<Node *, double> cost_map;
    cost_map[rrt->root] = 0;
    Node *qBest;
    double cost, dist, distBest;
    for (int i = 0; i < renderArea->rrt->max_iter; i++) {
        Node *qNew = rrt->getRandomNode();
        Node *qNearest = rrt->nearest(qNew->position);
        if (rrt->distance(qNew->position, qNearest->position) > rrt->step_size) {
            Vector2f newConfig = rrt->newConfig(qNew, qNearest);
            if (rrt->obstacles->isSegmentInObstacle(newConfig, qNearest->position)) {
                continue;
            }
            qNew = new Node;
            qNew->position = newConfig;
        }

        rrt->nearestNeighbors(qNew->position);
        qBest = qNearest;
        distBest = rrt->distance(qNew->position, qNearest->position);

        // look for shortest cost path to qNew
        for (auto node : rrt->nodes) {
            // compute the cost for qNew through node
            dist = rrt->distance(qNew->position, node->position);
            cost = cost_map[node] + dist;
            if (dist > rrt->step_size * 2) {
                break;
            }

            // if cost less than current lowest cost, check potential link
            if (cost < cost_map[qBest] + distBest) {
                if (rrt->obstacles->isSegmentInObstacle(node->position,
                                                        qNew->position)) {
                    continue;
                }
                qBest = node;
                distBest = dist;
            }
        }
        rrt->add(qBest, qNew, distBest);
        cost_map[qNew] = cost_map[qBest] + distBest;

        for (auto node : rrt->nodes) {
            dist = rrt->distance(qNew->position, node->position);
            cost = cost_map[node] + dist;

            // if cost less than current lowest cost, check potential link
            if (cost < cost_map[node]) {
                if (rrt->obstacles->isSegmentInObstacle(node->position,
                                                        qNew->position)) {
                    continue;
                }
                rrt->relink(node, qNew, dist);
                cost_map[node] = cost;
            }
        }
        if (rrt->reached()) {
            ui->statusBox->setText(tr("Reached Destination!"));
            break;
        }
    }
    Node *q;
    if (rrt->reached()) {
        q = rrt->lastNode;
    }
    else
    {
        // if not reached yet, then shortestPath will start from the closest node to end point.
        q = rrt->nearest(rrt->endPos);
        ui->statusBox->setText(tr("Exceeded max iterations!"));
    }
    // generate shortest path to destination.
    while (q != NULL) {
        rrt->path.push_back(q);
        q = q->parent;
    }
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    cout << "Duration " << duration << endl;
    renderArea->update();
}

/**
 * @brief Delete all obstacles, nodes and paths from the simulator.
 */
void MainWindow::on_resetButton_clicked()
{
    simulated = false;
    ui->statusBox->setText(tr(""));
    rrt->reset();
    rrt->initialize();
    renderArea->update();
}

MainWindow::~MainWindow()
{
    delete ui;
}
