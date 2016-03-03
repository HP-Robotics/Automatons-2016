package org.usfirst.frc.team2823.robot;

/* Use by setting a robot.boolean for whether or not the thread has started.
   If it hasn't started, then start it with:
        PiThread pi = new PiThread();
        pi.start();
   The getLast() method should return the last message the Pi sent to us.

*/

public class PiThread extends Thread {

        private String m_talk;
        private boolean m_running = true;

        public void run() {
            TalkToPi.rawCommand("HELLO");
            while (m_running) {
                try {
                    m_talk = TalkToPi.recv();
                    System.out.println(m_talk);
                } catch (Exception e) {
                    System.out.println("Except!");
                    break;
                }
            }
        }

        public void stopReading() {
            m_running = false;
            TalkToPi.close();
        }

        public String getLast() {
            return m_talk;
        }
}


