import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

def live_plot(shared, exit_flag):
    """
    Live plot process.
    shared = dict of multiprocessing.Value references from main
    exit_flag = mp.Value to signal exit
    """

    plt.style.use("ggplot")
    fig, axs = plt.subplots(3, 2, figsize=(16, 12))
    ax1, ax2, ax3, ax4, ax5, ax6 = axs.flatten()

    # --- Plot 1: Loadcells
    lc_line_r, = ax1.plot([], [], label="R Loadcell")
    lc_line_l, = ax1.plot([], [], label="L Loadcell")
    ax1.set_ylabel("Loadcell (N)")
    ax1.legend(loc="center left", bbox_to_anchor=(1, 0.5))

    # --- Plot 2: Pressures
    pr_labels = ["R","RR","L","LL","psRu","psRd","psLu","psLd"]
    pr_lines = [ax2.plot([], [], label=lab)[0] for lab in pr_labels]
    ax2.set_ylabel("Pressure (kPa)")
    ax2.legend(ncol=2, loc="center left", bbox_to_anchor=(1, 0.5))

    # --- Plot 3: IMUs
    imu_labels = ["Right Thigh","Right Shank","Left Thigh","Left Shank","Hip"]
    imu_lines = [ax3.plot([], [], label=lab)[0] for lab in imu_labels]
    ax3.set_ylabel("IMU angle (°)")
    ax3.legend(loc="center left", bbox_to_anchor=(1, 0.5))

    # --- Plot 4: FSRs
    fsr_labels = ["FSR R Heel","FSR R Toe","FSR L Heel","FSR L Toe"]
    fsr_lines = [ax4.plot([], [], label=lab)[0] for lab in fsr_labels]
    ax4.set_ylabel("FSR")
    ax4.legend(loc="center left", bbox_to_anchor=(1, 0.5))

    # --- Plot 5: Heel/Toe strikes
    ax5.set_ylim(0, 2)
    ax5.set_xlim(-0.5, 1.5)
    ax5.set_xticks([0, 1])
    ax5.set_xticklabels(["Left", "Right"])
    bars_heel = ax5.bar([0, 1], [0, 0], bottom=[0, 0], color=["none","none"], edgecolor=["none","none"])
    bars_toe  = ax5.bar([0, 1], [0, 0], bottom=[1, 1], color=["none","none"], edgecolor=["none","none"])
    heel_texts = [ax5.text(x, 0.5, "Heel", ha="center", va="center",
                           fontsize=20, color=c, visible=False)
                  for x,c in zip([0,1],["blue","red"])]
    toe_texts = [ax5.text(x, 1.5, "Toe", ha="center", va="center",
                          fontsize=20, color=c, visible=False)
                 for x,c in zip([0,1],["cyan","orange"])]
    ax5.set_ylabel("Contact")

    # --- Plot 6: Heel/support hierarchy
    ax6.set_ylim(0, 2)
    ax6.set_xlim(-0.5, 1.5)
    ax6.set_xticks([0, 1])
    ax6.set_xticklabels(["Left Heel Cycle","Right Heel Cycle"])
    heel_bars = ax6.bar([0,1],[0,0],width=0.6,color=["blue","red"],alpha=0.5)
    unsupp_bars = ax6.bar([0,1],[0,0],width=0.4,color=["purple","brown"],alpha=0.7)
    supp_bars   = ax6.bar([0,1],[0,0],width=0.2,color=["green","orange"],alpha=0.9)
    ax6.set_ylabel("Time (s)")
    handles = [heel_bars[0],unsupp_bars[0],supp_bars[0],
               heel_bars[1],unsupp_bars[1],supp_bars[1]]
    labels = ["Left Heel","Left Unsupport","Left Support",
              "Right Heel","Right Unsupport","Right Support"]
    ax6.legend(handles, labels, loc="center left", bbox_to_anchor=(1, 0.5))

    # Buffers
    t_buf, lc_bufs, pr_bufs, imu_bufs, fsr_bufs = [], [[] for _ in range(2)], [[] for _ in range(8)], [[] for _ in range(5)], [[] for _ in range(4)]
    time_window = 50 # 300

    def update(frame):
        if exit_flag.value:
            plt.close(fig)
            return

        # --- collect values from shared memory
        t = time.time()
        lc_vals = [shared["Lr"].value, shared["Ll"].value]
        pr_vals = [shared["R"].value, shared["RR"].value, shared["L"].value, shared["LL"].value,
                   shared["psRu"].value, shared["psRd"].value, shared["psLu"].value, shared["psLd"].value]
        imu_vals = [shared["IMU_11"].value, shared["IMU_22"].value, shared["IMU_33"].value,
                    shared["IMU_44"].value, shared["IMU_55"].value]
        fsr_vals = [shared["FSRrh"].value, shared["FSRrt"].value, shared["FSRlh"].value, shared["FSRlt"].value]
        heelL, toeL, heelR, toeR = shared["heelL"].value, shared["toeL"].value, shared["heelR"].value, shared["toeR"].value
        heelLp, heelRp = shared["HeelLp"].value, shared["HeelRp"].value
        unsuppL, unsuppR = shared["unsupportTimeL"].value, shared["unsupportTimeR"].value
        suppL, suppR = shared["supportTimeL"].value, shared["supportTimeR"].value

        # append buffers
        t_buf.append(t)
        for buf,v in zip(lc_bufs, lc_vals): buf.append(v)
        for buf,v in zip(pr_bufs, pr_vals): buf.append(v)
        for buf,v in zip(imu_bufs, imu_vals): buf.append(v)
        for buf,v in zip(fsr_bufs, fsr_vals): buf.append(v)
        t_buf[:] = t_buf[-time_window:]
        for buf in (lc_bufs+pr_bufs+imu_bufs+fsr_bufs): buf[:] = buf[-time_window:]

        # update lines
        for line,buf in zip([lc_line_r,lc_line_l], lc_bufs): line.set_data(t_buf, buf)
        for line,buf in zip(pr_lines, pr_bufs): line.set_data(t_buf, buf)
        for line,buf in zip(imu_lines, imu_bufs): line.set_data(t_buf, buf)
        for line,buf in zip(fsr_lines, fsr_bufs): line.set_data(t_buf, buf)
        for ax in [ax1,ax2,ax3,ax4]:
            ax.relim(); ax.autoscale_view()

        # --- Plot 5 Heel/Toe strikes
        if heelL:
            bars_heel[0].set_height(1); bars_heel[0].set_facecolor("blue")
            heel_texts[0].set_visible(True)
        else:
            bars_heel[0].set_height(0); bars_heel[0].set_facecolor("none")
            heel_texts[0].set_visible(False)

        if toeL:
            bars_toe[0].set_height(1); bars_toe[0].set_facecolor("cyan")
            toe_texts[0].set_visible(True)
        else:
            bars_toe[0].set_height(0); bars_toe[0].set_facecolor("none")
            toe_texts[0].set_visible(False)

        if heelR:
            bars_heel[1].set_height(1); bars_heel[1].set_facecolor("red")
            heel_texts[1].set_visible(True)
        else:
            bars_heel[1].set_height(0); bars_heel[1].set_facecolor("none")
            heel_texts[1].set_visible(False)

        if toeR:
            bars_toe[1].set_height(1); bars_toe[1].set_facecolor("orange")
            toe_texts[1].set_visible(True)
        else:
            bars_toe[1].set_height(0); bars_toe[1].set_facecolor("none")
            toe_texts[1].set_visible(False)

        # --- Plot 6 Heel/support
        heel_bars[0].set_height(heelLp)
        heel_bars[1].set_height(heelRp)
        unsupp_bars[0].set_height(unsuppL)
        unsupp_bars[1].set_height(unsuppR)
        supp_bars[0].set_height(suppL)
        supp_bars[1].set_height(suppR)

        return []

    ani = animation.FuncAnimation(fig, update, interval=1, blit=False)
    plt.show()
