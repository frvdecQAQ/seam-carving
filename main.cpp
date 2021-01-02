#include <iostream>
#include "SeamCarve.h"

int main() {
    auto seam_carve = std::make_shared<SeamCarve>(1024, 1024);
    seam_carve->set_img_in("../data/remove.jpg");
    //seam_carve->set_remove_mask("../data/remove_mask.jpg");
    seam_carve->set_energy_choice(kGrad);
    seam_carve->set_forward(false);
    //seam_carve->remove_object();
    //seam_carve->run(0, -100);
    seam_carve->show_img_out();
    seam_carve->store_img_out("../data/remove_result.jpg");
    return 0;
}
