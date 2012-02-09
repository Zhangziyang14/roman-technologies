# zonder extensie
ROOT = Assignment_1
BIB = biblio

$(ROOT).pdf: *.tex
	latex -interaction=nonstopmode $(ROOT).tex
	dvips -o $(ROOT).ps $(ROOT).dvi
	ps2pdf $(ROOT).ps

clean:
	rm *.dvi *.aux *.ps *.toc *.out *.log
