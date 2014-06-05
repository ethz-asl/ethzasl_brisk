import argparse
import ShelveWrapper as sw
import sm
import pickle 

startFrame = 1400
endFrame = 2700

def extractFrameIndices(s, startFrame, endFrame, min_deg):
  indices = [startFrame]
  mfA, T_w_a = s[startFrame]  
  T_a_w = T_w_a.inverse() 
  q = min_deg
  for b in range(startFrame + 1, endFrame+1):
    print 'at b: ', b
    mfB, T_w_b = s[b]
    T_a_b = T_a_w * T_w_b
    angle = sm.rad2deg(sm.R2rph(T_a_b.C())[2])
    if angle < 0.0:
      pangle = 360.0 + angle
    else:
      pangle = angle

    if pangle >= q:
      indices.append(b)
      q += min_deg

  if not endFrame in indices:
    indices.append(endFrame)

  return indices

def process(inputshelve, tag, min_deg):
  s = sw.ShelveDb(inputshelve)
  print 'extracting indices'
  idxs = extractFrameIndices(s, startFrame, endFrame, min_deg)
  pickle.dump(idxs, open('indices_' + tag + '_' + str(min_deg) + '.bin', 'w'))
  print 'done!'

def main():
    parser = argparse.ArgumentParser(description="")

    parser.add_argument("inputshelve", type=str, help="")
    parser.add_argument("tag", type=str, help="")
    parser.add_argument("min_deg", type=str, help="")

    args = parser.parse_args()

    inputshelve = str(args.inputshelve)
    tag = str(args.tag)
    min_deg = str(args.min_deg)

    process(inputshelve, tag, min_deg)

if __name__ == '__main__':
    main()
